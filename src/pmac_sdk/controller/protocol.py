"""Version 2 PMAC mailboxes. Addresses are 16-bit Modbus register addresses.

Sys.ModbusServerBuffer uses byte offsets (twice these addresses). A single
driver process owns the mailboxes; its feedback and motion calls share a lock.
"""

from dataclasses import dataclass
from contextlib import contextmanager
import time


PROTOCOL_VERSION = 2
PVT_REQUEST = 200
FEEDBACK_REQUEST = 208
HOMING_COMMAND = 220
HOMING_STATUS = 222


@dataclass(frozen=True)
class DriverStatus:
    pending: int
    accepted: int
    fault: int
    queued: int
    feedback_pending: int
    feedback_accepted: int
    version: int
    ready: int


@dataclass
class TransactionTiming:
    total_ms: float = 0.0
    read_ms: float = 0.0
    write_ms: float = 0.0
    sleep_ms: float = 0.0
    sleep_requested_ms: float = 0.0
    reads: int = 0
    writes: int = 0
    polls: int = 0
    sleeps: int = 0
    success: bool = False


class PMACProtocol:
    def __init__(self, modbus, timeout_s: float = 0.2, poll_interval_s: float = 0.001):
        if timeout_s <= 0 or poll_interval_s <= 0:
            raise ValueError("Protocol timeouts must be positive.")
        self.modbus = modbus
        self.timeout_s = timeout_s
        self.poll_interval_s = poll_interval_s
        self.sleep_fn = None  # None preserves the default time.sleep path.
        self.last_pvt_status: DriverStatus | None = None
        self.trace_enabled = False
        self.last_submit_timing: TransactionTiming | None = None
        self.last_snapshot_timing: TransactionTiming | None = None
        self._active_timing: TransactionTiming | None = None

    @contextmanager
    def _measure_transaction(self, kind):
        # Enter only while the mailbox transaction lock is held. No extra I/O.
        setattr(self, f"last_{kind}_timing", None)
        if not self.trace_enabled:
            yield
            return
        timing = TransactionTiming()
        self._active_timing = timing
        start = time.perf_counter()
        try:
            yield
            timing.success = True
        finally:
            timing.total_ms = (time.perf_counter() - start) * 1000
            setattr(self, f"last_{kind}_timing", timing)
            self._active_timing = None

    def _io(self, operation, address, payload):
        method = self.modbus.read_int32_array if operation == "read" else self.modbus.write_int32_array
        kwargs = {"address": address, "count" if operation == "read" else "values": payload}
        timing = self._active_timing
        if timing is None:
            return method(**kwargs)
        start = time.perf_counter()
        try:
            return method(**kwargs)
        finally:
            setattr(timing, operation + "s", getattr(timing, operation + "s") + 1)
            setattr(timing, operation + "_ms", getattr(timing, operation + "_ms") +
                    (time.perf_counter() - start) * 1000)

    def write(self, address: int, values: list[int]) -> None:
        if not self._io("write", address, values):
            raise ConnectionError(f"Modbus write rejected at address {address}")

    def status(self) -> DriverStatus:
        status = DriverStatus(*self._io("read", PVT_REQUEST, 8))
        if status.version != PROTOCOL_VERSION:
            raise RuntimeError(
                f"PMAC protocol {status.version} is incompatible; "
                "install the matching protocol-v2 PLC project before using this driver."
            )
        return status

    def wait(self, predicate, description: str, timeout_s: float | None = None):
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while True:
            timing = self._active_timing
            if timing is not None:
                timing.polls += 1
            result = predicate()
            if result:
                return result
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for {description}")
            if timing is None:
                (self.sleep_fn or time.sleep)(self.poll_interval_s)
            else:
                start = time.perf_counter()
                try:
                    (self.sleep_fn or time.sleep)(self.poll_interval_s)
                finally:
                    timing.sleeps += 1
                    timing.sleep_requested_ms += self.poll_interval_s * 1000
                    timing.sleep_ms += (time.perf_counter() - start) * 1000

    @staticmethod
    def next_token(acknowledged: int) -> int:
        # One-byte tokens avoid a partially updated multi-byte trigger in PLC.
        return acknowledged % 255 + 1

    def snapshot(self) -> tuple[list[int], tuple[int, ...]]:
        """Request fresh positions and homing status, frozen until next request."""
        with self.modbus.transaction_lock, self._measure_transaction("snapshot"):
            status = self.status()
            if status.feedback_pending:
                self.wait(lambda: not self.status().feedback_pending, "previous feedback snapshot")
                status = self.status()
            token = self.next_token(status.feedback_accepted)
            self.write(FEEDBACK_REQUEST, [token])

            def ready():
                status = self.status()
                return not status.feedback_pending and status.feedback_accepted == token

            self.wait(ready, "fresh PMAC feedback snapshot")
            positions = self._io("read", 10, 5)
            homing = tuple(self._io("read", HOMING_STATUS, 5))
            return positions, homing

    def require_stream_ready(self) -> DriverStatus:
        status = self.status()
        if status.fault:
            raise RuntimeError(f"PMAC PVT fault {status.fault}; restart the driver after recovery.")
        if not status.ready:
            raise RuntimeError("PMAC PVT program is not ready.")
        return status

    def submit_pvt(self, positions: list[int], velocities: list[int], duration: int) -> None:
        self.last_pvt_status = None
        with self.modbus.transaction_lock, self._measure_transaction("submit"):
            self.wait(lambda: not self.require_stream_ready().pending, "previous PVT submission")
            status = self.require_stream_ready()
            token = self.next_token(status.accepted)
            self.write(0, positions)
            self.write(40, [duration])
            self.write(50, velocities)
            self.write(PVT_REQUEST, [token])

            def accepted():
                status = self.require_stream_ready()
                if status.pending:
                    return False
                if status.accepted != token:
                    raise RuntimeError("PMAC cleared the PVT request without accepting this frame.")
                return status

            # An ambiguous timeout is never retried: execution may have begun.
            self.last_pvt_status = self.wait(accepted, "PVT acceptance acknowledgement")
