#!/usr/bin/env python3
"""Run only the fake-linked test and remove only IPC resources it created."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def ipc_table(kind):
    lines = Path(f"/proc/sysvipc/{kind}").read_text().splitlines()
    return [dict(zip(lines[0].split(), line.split())) for line in lines[1:]]


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: run_fake.py /path/to/igh_fake_smoke")
    binary = Path(sys.argv[1]).resolve()
    old_sem_ids = {row["semid"] for row in ipc_table("sem")}
    with tempfile.TemporaryDirectory(prefix="continuum-igh-offline-") as directory:
        env = dict(os.environ, FAKE_EC_HOMEDIR=directory,
                   FAKE_EC_NAME="ContinuumOffline", FAKE_EC_PREFIX="/ContinuumOffline")
        # No LD_LIBRARY_PATH redirection to masquerade a fake library as real.
        process = subprocess.Popen([str(binary)], env=env, text=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            output, _ = process.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
            print(output)
            raise RuntimeError("fake smoke timed out")
        finally:
            # RtIPC intentionally keeps SysV resources after exit. Creator PID
            # and owner restrict removal to this child; never use `ipcrm -a`.
            owned = [r for r in ipc_table("shm") if int(r["cpid"]) == process.pid
                     and int(r["cuid"]) == os.getuid()]
            keys = {r["key"] for r in owned}
            for r in ipc_table("sem"):
                if r["key"] in keys and r["semid"] not in old_sem_ids and int(r["cuid"]) == os.getuid():
                    subprocess.run(["ipcrm", "-s", r["semid"]], check=True)
            for r in owned:
                subprocess.run(["ipcrm", "-m", r["shmid"]], check=True)
        (binary.parent / "fake-smoke.log").write_text(output)
        print(output if process.returncode else "\n".join(line for line in output.splitlines()
              if line.startswith(("PASS:", "No physical"))))
        return process.returncode


if __name__ == "__main__":
    raise SystemExit(main())
