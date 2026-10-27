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

仿真保持 `scene_main.xml` 和原视角，初始插入量由编码器零位偏置换算，
两段弯曲从零开始，首次主手输入仅建立参考。
仿真保留已确认的局部标定：输入平移先变换为 `[x,z,-y]`；旋转增量先反转
局部手柄长轴 X，再经过既有轴交换，合成为 `[rx,ry,rz] → [rz,ry,rx]`，
之后送入参考旋转映射与 IK。这不是世界坐标 Rz 的单独取反。
`omega_reference_sim.py --rotation-map original` 可恢复未交换的参考旋转输入。

## 平移与旋转的标定关系

仿真平移默认复用真机链路的配置矩阵（`--translation-map robot-branch`），
同时保留上述独立的输入基变换。因此不能直接声明相同原始位移在两条链路中
数值一致。配置矩阵由 `omega_teleop.yaml` 的 device→WORLD 映射、`robot_interface` 的
WORLD→内部 IK 帧，以及内部帧到参考项目基座帧的换名复合而成，读取配置，
不改真机链路。`--translation-map reference` 可恢复参考项目原映射。

当前 `omega_teleop.yaml` 平移增益为 `[0.45,0.45,0.45]`，旋转增益为
`[0.8,0,0.8]`。仿真使用矩阵旋转输入，不直接套用真机腕关节旋转增益。
`tests/test_branch_translation_match.py` 用经过标定基变换的输入独立运行真机 IK，
比较其与仿真末端位移的方向（<1°）和幅值，固定当前标定关系。

旋转按构造仍不对应，不是参数未调：真机的旋转命令只来自 Omega 腕部关节角增量
（`dhd.getOrientationRad`），3×3 旋转矩阵在真机侧仅参与位置偏移；仿真只用
3×3 矩阵的相邻增量。广播消息不含腕部关节角，仿真因此拿不到真机使用的信号。
真机 WORLD Ry 被禁用（gain 0，`max_rotation_y` 0）。仿真的倾转仅供观察，
不要据此判断真机姿态。
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
  --interface-config config/robot_interface.yaml \
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
./.venv/Scripts/python.exe apps/omega_reference_sim.py \
  --interface-config config/robot_interface.yaml
```

仿真初始插入量同样使用 `--interface-config` 中的 `initial_position.insertion_zero_pulses`。
它不会自动读取真机编码器；默认假定启动读数为 0。若实际读数不是 0，启动时填写：

```bash
./.venv/Scripts/python.exe apps/omega_reference_sim.py \
  --interface-config config/robot_interface.yaml --initial-axis5-pulses 12345
```

将 `12345` 替换为真机 driver 启动打印的第五轴 `current` 读数，且两边使用同一份
interface 配置。启动时会打印计算后的 `d`（毫米），应与 driver 的初始插入量一致。
例如零位偏置为 `-1255635`、读数为 0 时，仿真从约 20 mm 插入量开始。
这是启动姿态对齐，不会持续同步真机反馈；仿真仍然独立求解。偏置为 `null` 时
沿用当前位置作为零位，初始插入量为 0。越过模型插入范围时拒绝启动。
当前主配置偏置是 `-7000000`，读数为 0 时约为 111.5 mm；trial 配置仍为 0。
不带 `--interface-config` 的仿真默认读 trial 文件，不能混用这两个零位。

4. 查看器，仍使用上一版模型：

```bash
./.venv/Scripts/python.exe apps/view_robot_mujoco.py --source ideal --fps 60
```

查看器应显示 `LIVE ideal (independent_reference_ik)`。仿真和查看器可随时退出、
重启，不影响 LeRobot 真机控制。仅看仿真时可以让真机 driver 保持 dry-run。

旧的 `omega_reference_input.py` 与 driver 的 `--input-mode omega-reference`
实验入口已清理，统一使用以上并行方案。

## 复位：两段弯曲归零，进给不动（straighten）

**先停掉遥操作**(终端 2 的 LeRobot 进程 Ctrl+C),然后在**驱动终端**(终端 1)
按 `h`,驱动把两段弯曲角 θ_a/θ_c 按斜坡送到 0,进给 d 和两个弯曲平面角 φ 保持不动。
只有四个旋转轴运动,管子不会进退。按 `r` 则只清指令偏移、不动电机。

驱动不用退出,它持有 PMAC 连接,复位由它自己执行。遥操作可以随后直接重连。
按键监听在驱动进程里(`--reset-keys` / `--no-reset-keys` 开关,
`--straighten-key`、`--recenter-key` 改键位),与遥操作链路**不耦合**:
LeRobot 插件没有复位按键监听,也不发复位消息。
当前监听使用 pynput 全局键盘事件，不按终端焦点隔离；需要关闭时用
`--no-reset-keys`。这里指复位键，LeRobot 的空格离合监听仍由原插件负责。

不要在遥操作还在跑的时候按:客户端仍在持续发自己的位姿指令,会和复位抢控制权。
驱动启动时会打印这条提醒。

```python
from continuum_sdk.transport import build_straighten_message
command_socket.send_json(build_straighten_message(sequence))
```

由**驱动自己执行**,因为只有它持有 PMAC 连接。不要另起进程点动:
`safe_boot_and_home()` 会触发 PLC 复位,在驱动运行时启动会搞乱正在跑的 PVT 会话。

目标是运动学零位,不依赖 `reference_pulses`(那个不准)。整条斜曲线在**动之前**
先过一遍 `RotaryTargetGuard`,超阈值直接拒绝并提示降低
`execution.straighten_max_theta_rate_rad_s`(默认 0.5 rad/s,90° 约 3 秒)。
斜坡用 smoothstep,峰值是平均的 1.5 倍,时长已按峰值反算。

复位完成后驱动会自动 recenter 并**闸住算子输入**:手没动之前旧偏移不生效,
否则持续流入的旧指令会立刻把末端弯回原状。手一动就从新位姿恢复跟随。
状态消息里有 `straighten_count` 和 `straighten_active`。

## 仅复位数据 / recenter

遥操作过程中累积的是**相对量**：算子的 WORLD 偏移、滤波器的 target/applied 状态、
输入跟踪锚点。复位这些数据不需要动电机。遥操作中按 `R`，或发一条 `recenter` 消息：

```python
from continuum_sdk.transport import build_recenter_message
command_socket.send_json(build_recenter_message(sequence))
```

驱动端会在**同一个控制周期内**做三件事：清掉滤波器的累积偏移、把 `center_p/center_r`
重新锚到当前 FK 位姿、重置 IK 限位监视。三者必须同周期完成，否则下一次 `step()`
会把刚丢弃的偏移当成运动指令发出去。

消息不带 `action`，所以**不会命令任何运动**；编码器和 `base_pulses` 都不变。
离线验证:偏移从 0.004 归零后,目标脉冲在模拟的 1 秒空闲内逐字节不变。

复位后滤波器保持 held 状态。这是故意的:算子手柄可能还停在远离新原点的位置,
直接接管会导致跳动。下一次输入先成为新锚点,然后才开始跟随。
状态消息里有 `recenter_count` 可供界面确认复位生效。

基准脉冲位(`reference_pulses`)按你说的本来就不准,所以这条路径完全不依赖它。

### 为什么不回基准脉冲位

曾经写过一个把电机送回 `initial_position.reference_pulses` 的脚本,已删除:
基准位估计不准,朝它运动可能把机器人送到错的地方。现在复位的目标是
运动学零位(θ=0),与基准脉冲无关。

## 开机姿态不直时的手动电机复位

`apps/keyboard_reset_motor.py`,按住即动、松手即停,直接控制五个逻辑电机。

```bash
./.venv/Scripts/python.exe apps/keyboard_reset_motor.py            # 空跑
./.venv/Scripts/python.exe apps/keyboard_reset_motor.py --execute
```

`alpha1` `q/w`、`alpha2` `a/s`、`alpha3` `z/x`、`alpha4` `e/r`、进给 `c/v`。
ctrl 粗调、shift 细调,空格冻结,`p` 打印,esc 退出并停机。
默认 20 deg/s(ctrl 40),行程相对启动位置 ±90°/±30 mm。

**为什么必须是电机级,不能用关节角或一键复位**:`safe_boot_and_home()` 把当前
编码器读数记为基准,所以末端实际弯着开机时,模型算出的 θ_a=θ_c=0,认为已经是直的。
此时关节空间和笛卡尔指令都失去意义,`StraightenRamp.already_straight` 会是 True、
一键复位什么都不做。这种状态下唯一可信的量就是电机脉冲,所以恢复只能在这一层做。
(`tests/test_keyboard_reset_motor.py::test_a_bent_startup_pose_reads_as_zero_bend` 固定了这个前提。)

每根牵引丝都同时耦合两段,所以单个电机一般会同时改变两段形状。例如从某个弯曲
姿态让 alpha1 动 +2°,近段 -4.41°、远段 -2.90°;alpha2 动 +2° 则两段反向
(近段 +3.47°、远段 -3.99°)。因此界面每秒打印一次由已走脉冲反推的弯曲量
(`inferred bend A/C`),用于判断形状被推往哪个方向——它是相对启动姿态的相对量,
开机姿态未知时不代表绝对真值。

走到看起来直了之后,重启驱动服务让它把这个姿态记为新基准,再开始遥操作。

注意本脚本自己打开 PMAC(`safe_boot_and_home()` 会触发 PLC 复位),
**不能在驱动服务运行时启动**,否则会搞乱正在跑的 PVT 会话。
遥操作过程中的复位请用上面的 `H` 键。

关节空间点动版(按 θ/φ 控制)已删除:在开机不直的场景里它不可用,
而那正是手动复位唯一需要用到的场景。

空格是冻结目标而非中止 PVT：`stop_motion()` 会结束 PVT 会话，恢复需要整体重启，
所以保持用的是"目标不变、速度 0"的帧，与驱动服务的 holding 一致。

注意 `safe_boot_and_home()` 不做物理归位，它把**当前**位置记为编码器基准
（`robot_api.py:34`）。因此本脚本的限幅原点是启动时的姿态，不是机械零点；
零点由人眼判断。若要把走到的姿态固化为基准，另外调用
`set_current_as_absolute_zero()`。

## 离线验证

2026-10-27 收敛检查：`tests/` 共 435 项通过；全链路 dry-run 通过。
测试预期已按当前操作确认的映射和配置增益更新，未为迁就旧测试回退控制行为。

测试覆盖原 LeRobot 动作与离合不变、广播失败不打断动作链路、独立子进程
接收原始位姿并输出理想关节状态、断流停止发布、插入初值换算，以及当前
标定下的平移方向与增益、旋转正负和离合行为。没有启动硬件。

```bash
./.venv/Scripts/python.exe -m pytest tests -q
```

pytest 默认仅收集 `tests/`，因此在仓库根运行 `python -m pytest -q` 也可。
`apps/test_*.py` 和集成插件中的 `smoke_test.py` 是手动调试入口，不自动执行。
已移除 3 个仍调用旧点位接口的电机示例，以及早期手动读主手的
`tests/test_omega.py`；协议、轴映射、IK、PVT 保护及遥操作回归测试保留。
