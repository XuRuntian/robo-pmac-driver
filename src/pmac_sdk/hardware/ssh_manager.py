import re
import shlex
import time

import paramiko


class PMACHardwareManager:
    """PMAC lifecycle commands; a failed command must stop the boot sequence."""

    def __init__(self, ip, user, password):
        self.ip = ip
        self.user = user
        self.password = password

    def send_gpascii_commands(self, commands: list, delay=0.05):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(
                hostname=self.ip, port=22, username=self.user, password=self.password,
                timeout=5, banner_timeout=5, auth_timeout=5,
            )
            for description, command in commands:
                _, stdout, stderr = ssh.exec_command(
                    "printf '%s\\n' " + shlex.quote(command) + " | gpascii -2",
                    timeout=5,
                )
                output = stdout.read().decode(errors="replace").strip()
                error = stderr.read().decode(errors="replace").strip()
                exit_code = stdout.channel.recv_exit_status()
                # gpascii returns exit code 1 on normal EOF after piped stdin.
                # Therefore exit_code == 1 is not a PMAC command failure.
                pmac_error = re.search(
                    r"(?:\bERR\d*\b|\berror\s*(?:#\d+)?\b|ILLEGAL\s+CMD)",
                    output,
                    re.I,
                )

                if error or pmac_error or exit_code not in (0, 1):
                    raise RuntimeError(
                        f"PMAC command failed ({description}): {command}; "
                        f"exit={exit_code}, stdout={output!r}, stderr={error!r}"
                    )
                if delay:
                    time.sleep(delay)
        finally:
            ssh.close()

    def reset_with_plc4(self):
        # Completion is acknowledged through register 216, never a fixed sleep.
        self.send_gpascii_commands([("Reset PMAC through PLC 4", "enable plc 4")], delay=0)

    def init_motors(self):
        self.reset_with_plc4()

    def prepare_motors(self):
        self.reset_with_plc4()

    def start_prog(self):
        self.send_gpascii_commands([
            ("Map axes", "&1 #1->X #2->Y #3->Z #4->A #5->B"),
            ("Start PVT program", "&1 b1r"),
        ])

    def stop_pvt(self):
        # Abort also discards the coordinate system's already-planned motion.
        # Freeze the producer before clearing the application ring buffer.
        self.send_gpascii_commands([
            ("Stop PVT reception", "disable plc 2"),
            ("Abort coordinate system motion", "&1A"),
            ("Mark PVT stopped", "PVT_Ready=0"),
            ("Clear write index", "PVT_WriteIdx=0"),
            ("Clear read index", "PVT_ReadIdx=0"),
            ("Clear queue count", "PVT_Count=0"),
            ("Clear stream watchdog", "PVT_HasData=0"),
            *[("Clear PVT request", f"Sys.ModbusServerBuffer[{index}]=0")
              for index in range(400, 404)],
            ("Resume feedback", "enable plc 2"),
        ])
