# 调试记忆

## 全链路验证

使用 `apps/debug_full_chain.py` 验证 `外部笛卡尔目标 -> 坐标变换 -> DLS IK -> tendon/motor 映射 -> PMAC 轴顺序/符号 -> 脉冲 -> 反馈反解 -> FK`。默认 dry-run；只有确认打印的逻辑轴到物理轴映射后才使用 `--execute`。

`PMACConfig` 当前映射为逻辑 `[alpha1, alpha2, alpha3, alpha4, d]` 到物理 `[2, 1, 3, 4, 5]`，符号 `[+, -, -, -, +]`。任何新调试程序必须通过 `ContinuumAxisMapper`，不能按数组下标直接生成 PMAC 脉冲。

外部平移和旋转都采用 X/Y/Z（右/插入/向上）到 IK 内部 X/Y/Z（右/向下/插入）的同一基变换 `xzy`、符号 `[1,-1,1]`，即 `[rx, ry, rz] -> [rx, -rz, ry]`。当前机构在连续体坐标系不支持独立 Rz roll，因此世界坐标系的 Ry 禁用；世界 Rx/Rz 可用于 tip-axis tilt。键盘姿态测试使用 U/J=世界 Rx、I/K=世界 Rz。

当前硬件测试采用牵引孔半径 `0.00215 m`、线轴直径 `0.012 m`；这两个参数必须与 `tdrc_robot_system/scripts/hardware/interactive_pmac_joint_test.py` 保持一致。

`tdrc_robot_system` 的 bridge 接收弧度目标时必须同时处理 `base_positions`、`axis_order` 和 `axis_signs`；不能把弧度直接转换成绝对脉冲。
