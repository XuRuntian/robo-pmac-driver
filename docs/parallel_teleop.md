# LeRobot 真机与独立 MuJoCo 并行（当前方案）

```
Omega（仅由 LeRobot 打开一次）
  ├─ 原 LeRobot 映射/增益/离合 → Cartesian driver → IK/PVT → 真机
  └─ 原始位置/旋转矩阵/离合 PUB :5558
        → omega_reference_sim.py → 参考映射/IK → PUB :5557 → 原 MuJoCo 模型
```

真机仍使用已经调好的 LeRobot 链路。新增广播为可选功能，发送不等待仿真，
无人订阅时丢弃样本；广播初始化或发送失败会警告并停用该功能，原动作继续。
仿真不发送任何机器人指令，不打开 Omega，不依赖 driver 或 PMAC。

仿真保持 `scene_main.xml` 和原视角，从模型零姿态开始，首次输入仅建立主手参考。
仿真入口默认将 Omega 相邻样本的局部旋转增量 Rx/Rz 交换，再反转交换后的
Rz，即 `[rx,ry,rz] → [rz,ry,-rx]`，再送入参考映射与 IK；Ry、平移及真机链路不受影响。
`omega_reference_sim.py --rotation-map original` 可恢复未交换的参考旋转输入。
参考项目的映射和 IK 保留；离合状态来自 LeRobot，同一次按键同时冻结两条链路。
断流超过 0.2 秒暂停求解，查看器随后显示 STALE；恢复时重新建立主手参考。
两条链路各自求解，显示的是独立理想运动，不是实机指令或编码器的严格复现。

## 启动

退出旧的 `omega_reference_input.py`、`run_single_arm_teleop.py` 和
`--input-mode omega-reference` driver。不要同时让两个进程打开 Omega。
以下命令使用 Git Bash。driver、仿真、查看器均在 `robo-pmac-driver` 目录运行。

1. 真机 driver 恢复原入口。下例不连接 PMAC，实际测试时添加 `--execute`。
   不加 `--input-mode omega-reference`，也不要占用仿真的 5557 端口。

```bash
./.venv/Scripts/python.exe apps/continuum_driver_server.py \
  --interface-config config/robot_interface_stop_priority_trial.yaml \
  --wait-mode windows-highres --live-debug \
  --latency-csv "outputs/teleop_parallel_$(date +%Y%m%d_%H%M%S).csv"
```

2. 原来的 LeRobot 环境、原来的遥操作命令和参数保留，只增加：

```bash
--teleop.raw_pose_endpoint=tcp://127.0.0.1:5558
```

例如（在原 LeRobot 环境运行）：

```bash
lerobot-teleoperate \
  --robot.type=continuum_pmac --robot.id=continuum_robot \
  --robot.remote_ip=127.0.0.1 \
  --teleop.type=omega_continuum --teleop.id=omega_master \
  --teleop.omega_config=D:/project/surgical_continuum_robot/robo-pmac-driver/config/omega_teleop.yaml \
  --teleop.raw_pose_endpoint=tcp://127.0.0.1:5558
```

插件必须是本仓库修改后的版本。若提示参数不存在，检查原 LeRobot 环境的
安装路径；可在该环境中更新本地可编辑安装（不要在 driver 的 Python 3.10 环境安装 LeRobot）：

```bash
uv pip install --python D:/project/lerobot/.venv/Scripts/python.exe \
  -e D:/project/surgical_continuum_robot/robo-pmac-driver/integrations/lerobot_robot_continuum \
  -e D:/project/surgical_continuum_robot/robo-pmac-driver/integrations/lerobot_teleoperator_omega_continuum
```

3. 独立仿真 IK：

```bash
./.venv/Scripts/python.exe apps/omega_reference_sim.py
```

4. 查看器，仍使用上一版模型：

```bash
./.venv/Scripts/python.exe apps/view_robot_mujoco.py --source ideal --fps 60
```

查看器应显示 `LIVE ideal (independent_reference_ik)`。仿真和查看器可随时退出、
重启，不影响 LeRobot 真机控制。仅看仿真时可以让真机 driver 保持 dry-run。

旧的 `omega_reference_input.py` 与 driver 的 `--input-mode omega-reference`
实验入口已清理，统一使用以上并行方案。

## 离线验证

测试覆盖原 LeRobot 动作与离合不变、广播失败不打断动作链路、独立子进程
接收原始位姿并输出理想关节状态、断流停止发布。没有启动硬件。
