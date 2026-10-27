"""Read a fixed set of PMAC parameters over SSH; no motion or assignments.

Does not instantiate the robot controller or use the Modbus mailbox.
Results are sequential observations, not synchronized servo samples.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shlex

from pmac_sdk.core.config_model import PMACConfig


QUERIES = tuple(
    f"Motor[5].{field}"
    for field in (
        "Pos",
        "DesPos",
        "ActPos",
        "ClosedLoop",
        "AmpEna",
        "ServoCtrl",
        "Ctrl",
        "pDac",
        "pEnc",
        "PosSf",
        "MaxSpeed",
        "InvAmax",
        "InvDmax",
        "InvJmax",
        "JogSpeed",
        "MinPos",
        "MaxPos",
        "FatalFeLimit",
        "WarnFeLimit",
        "Servo.Kp",
        "Servo.Ki",
        "Servo.Kvfb",
        "Servo.Kvff",
        "Servo.Kaff",
        "Servo.MaxInt",
        "Servo.MaxPosErr",
    )
) + (
    "Axis5_MaxStep", "Axis5_MaxVel", "Coord[1].SegMoveTime", "Coord[1].LHDistance",
    # Project-specific fifth-slave aliases; undefined aliases fail the snapshot.
    "Slave_1005_Diamond_E_1005_6060_0_Modesofoperation",
    "Slave_1005_Diamond_E_1005_6061_0_Modesofoperationdisplay",
)


def read_command():
    # No user-supplied gpascii expressions, assignments, lifecycle or jog commands.
    return (
        "printf '%s\\n' " + " ".join(shlex.quote(q) for q in QUERIES) + " | gpascii -2"
    )


def capture(config, output, client_factory=None):
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve before connecting, so an existing capture is never overwritten.
    with output.open("x", encoding="utf-8") as log:
        import paramiko

        client = (client_factory or paramiko.SSHClient)()
        record = {
            "host": config.ip,
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "queries": QUERIES,
            "note": "Read-only sequential observations; not a synchronized servo snapshot. "
            "Symbolic globals may be returned under P-variable names. "
            "No credentials are recorded.",
        }
        try:
            client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(
                hostname=config.ip,
                port=22,
                username=config.ssh_user,
                password=config.ssh_pass,
                timeout=5,
                banner_timeout=5,
                auth_timeout=5,
                look_for_keys=False,
                allow_agent=False,
            )
            _, stdout, stderr = client.exec_command(read_command(), timeout=10)
            record["stdout"] = stdout.read().decode(errors="replace")
            record["stderr"] = stderr.read().decode(errors="replace")
            record["exit_code"] = stdout.channel.recv_exit_status()
            # gpascii commonly exits 1 on normal stdin EOF.
            record["complete"] = (
                record["exit_code"] in (0, 1)
                and not record["stderr"]
                and len(re.findall(r"^[^\r\n=]+=[^\r\n]+$", record["stdout"], re.M))
                == len(QUERIES)
                and not re.search(
                    r"\bERR\d*\b|\berror\b|ILLEGAL\s+CMD", record["stdout"], re.I
                )
            )
        except Exception as exc:
            record["complete"] = False
            # Exception messages may contain connection data; store only the type.
            record["connection_error_type"] = type(exc).__name__
        finally:
            client.close()
            record["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            json.dump(record, log, ensure_ascii=False, indent=2)
        if not record["complete"]:
            raise RuntimeError(f"Readback incomplete; inspect {output}")
    return record


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pmac-ip", default="192.168.0.200")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    capture(PMACConfig(ip=args.pmac_ip), args.output)
    print(f"Read-only parameter snapshot saved: {args.output}")
