"""Version 2 PMAC mailboxes. Addresses are 16-bit Modbus register addresses.

Sys.ModbusServerBuffer uses byte offsets (twice these addresses). A single
driver process owns the mailboxes; its feedback and motion calls share a lock.
"""

from dataclasses import dataclass
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


class PMACProtocol:
    def __init__(self, modbus, timeout_s: float = 0.2, poll_interval_s: float = 0.001):
        if timeout_s <= 0 or poll_interval_s <= 0:
            raise ValueError("Protocol timeouts must be positive.")
        self.modbus = modbus
        self.timeout_s = timeout_s
        self.poll_interval_s = poll_interval_s

    def write(self, address: int, values: list[int]) -> None:
        if not self.modbus.write_int32_array(address=address, values=values):
            raise ConnectionError(f"Modbus write rejected at address {address}")

    def status(self) -> DriverStatus:
        status = DriverStatus(*self.modbus.read_int32_array(PVT_REQUEST, 8))
        if status.version != PROTOCOL_VERSION:
            raise RuntimeError(
                f"PMAC protocol {status.version} is incompatible; "
                "install the matching protocol-v2 PLC project before using this driver."
            )
        return status

    def wait(self, predicate, description: str, timeout_s: float | None = None):
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while True:
            result = predicate()
            if result:
                return result
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for {description}")
            time.sleep(self.poll_interval_s)

    @staticmethod
    def next_token(acknowledged: int) -> int:
        # One-byte tokens avoid a partially updated multi-byte trigger in PLC.
        return acknowledged % 255 + 1

    def snapshot(self) -> tuple[list[int], tuple[int, ...]]:
        """Request fresh positions and homing status, frozen until next request."""
        with self.modbus.transaction_lock:
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
            positions = self.modbus.read_int32_array(10, 5)
            homing = tuple(self.modbus.read_int32_array(HOMING_STATUS, 5))
            return positions, homing

    def require_stream_ready(self) -> DriverStatus:
        status = self.status()
        if status.fault:
            raise RuntimeError(f"PMAC PVT fault {status.fault}; restart the driver after recovery.")
        if not status.ready:
            raise RuntimeError("PMAC PVT program is not ready.")
        return status

    def submit_pvt(self, positions: list[int], velocities: list[int], duration: int) -> None:
        with self.modbus.transaction_lock:
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
                return True

            # An ambiguous timeout is never retried: execution may have begun.
            self.wait(accepted, "PVT acceptance acknowledgement")
