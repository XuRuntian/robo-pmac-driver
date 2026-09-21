"""Save held, stable encoder feedback from the running continuum driver.

Stop the Omega/keyboard client and any other state receiver first; keep the
driver running. This is a PULL receiver on the existing state stream and sends
no robot commands. It never modifies robot_interface.yaml.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import zmq


def held_feedback(message):
    if message.get("protocol_version") != 1 or message.get("kind") != "state":
        return None
    status = message.get("status", {})
    if not all(status.get(key) is True for key in ("execute", "feedback_valid", "watchdog_holding")):
        return None
    values = status.get("feedback_pulses")
    if not isinstance(values, list) or len(values) != 5:
        return None
    if not all(type(v) is int for v in values) or not any(values):
        return None
    stamp = message.get("timestamp_ns")
    if not isinstance(stamp, int) or abs(time.time_ns() - stamp) > 2_000_000_000:
        return None
    return values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--remote-ip", default="127.0.0.1")
    parser.add_argument("--state-port", type=int, default=5556)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    output = args.output or Path("logs") / ("straight_reference_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    if output.exists():
        parser.error(f"File already exists: {output}; choose another output path.")
    endpoint = f"tcp://{args.remote_ip}:{args.state_port}"
    print("Waiting for held, stable hardware feedback. Stop teleop; keep driver running.")
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.connect(endpoint)
    deadline = time.monotonic() + 15.0
    samples = []
    stable_since = None
    try:
        while time.monotonic() < deadline:
            if not socket.poll(500):
                samples.clear()
                stable_since = None
                continue
            message = socket.recv_json()
            values = held_feedback(message)
            if values is None:
                samples.clear()
                stable_since = None
                continue
            samples.append(values)
            if any(max(axis) - min(axis) > 100 for axis in zip(*samples)):
                samples = [values]
                stable_since = None
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            if now - stable_since < 1.0 or len(samples) < 5:
                continue
            record = {
                "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
                "source": endpoint,
                "description": "Operator-observed approximately straight pose; not an automatic geometry calibration.",
                "physical_axis_order": [1, 2, 3, 4, 5],
                "reference_pulses": values,
                "observed_spread_pulses": [max(axis)-min(axis) for axis in zip(*samples)],
                "driver_state": message,
            }
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open("x", encoding="utf-8") as file:
                json.dump(record, file, ensure_ascii=False, indent=2)
                file.write("\n")
            print(f"Saved: {output.resolve()}")
            print(f"reference_pulses: {values}")
            print("Recorded only. Existing startup reference/configuration remains unchanged.")
            return
        raise SystemExit("No stable held hardware feedback within 15 s; no reference saved.")
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    main()
