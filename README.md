# robo-pmac-driver

连续体机器人的 Python 上位机：末端目标 → DLS IK → 拉索/电机映射 → PMAC PVT。

当前服务入口为 `apps/continuum_driver_server.py`，默认 dry-run；`--execute` 才连接硬件。配套控制器工程为 `surgical_continuum_robot_pmac`。

2026-09-08 起，硬件通信使用协议 v2，需要同时更新 Python 和 PMAC 工程。启动、反馈快照、PVT 确认、回零状态及离线测试说明见 [PMAC 协议与生命周期](docs/pmac_protocol_v2.md)。

LeRobot/Omega 的连接与使用见 [LeRobot integration](docs/lerobot_integration.md)。旧 `move_joints()` 点位协议未在配套 PLC 中实现，当前会明确拒绝；使用 PVT 服务或 PVT 应用入口。
