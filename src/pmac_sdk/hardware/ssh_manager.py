import paramiko
import time

class PMACHardwareManager:
    """管理 PMAC 底层系统级操作 (上下电、PLC启停)"""
    def __init__(self, ip, user, password):
        self.ip = ip
        self.user = user
        self.password = password

    def send_gpascii_commands(self, commands: list, delay=0.5):
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            ssh.connect(hostname=self.ip, port=22, username=self.user, password=self.password, timeout=10)
            print("🟢 SSH 连接成功。")
            
            for desc, cmd in commands:
                print(desc)
                stdin, stdout, stderr = ssh.exec_command(f"echo '{cmd}' | gpascii -2")
                
                # 【关键还原】：物理阻塞，确保 PMAC 消化完毕
                output = stdout.read().decode().strip()
                error = stderr.read().decode().strip()
                
                if error:
                    print(f"❌ 指令 [{cmd}] 执行出错: {error}")
                else:
                    print(f"✅ 响应: {output}")
                
                time.sleep(delay)
            print("✨ 硬件序列执行完毕")
        except Exception as e:
            print(f"❌ 初始化严重错误: {e}")
        finally:
            ssh.close()
            
    def init_motors(self):
        print(f"🔌 [系统初始化] 正在通过 SSH 唤醒 PMAC ({self.ip})...")
        self.send_gpascii_commands([
            ("🛑 正在停用 PLC 2...", "disable plc 2"),
            ("⚡ 正在执行电机上电 (#1..5k)...", "#1..5j/"),
            ("🔄 正在重新启用 PLC 2...", "enable plc 2")
        ])