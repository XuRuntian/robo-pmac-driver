# 键盘控制迟滞取证

## 当前检查点与下一阶段（2026-10-09）

本节是当前状态，后文按时间保留历史分析与测试结果。本检查点是可回退的调试版本，不代表 Omega 延迟验收通过或所有边界条件通过。

### 保留的代码与配置

- driver 可选高精度等待、协议内部计时、延迟 CSV 与离线分析；默认等待仍为 system，实机调试显式指定 windows-highres。
- 独立计算频率 benchmark 与 Windows 等待实验；测得的计算频率不能当作整条链路最高频率。
- 独立插入轴速度预算及截断诊断字段。0.01 m/s 保留既有上位机预算，与 live PLC 3000 counts/段取较小值，未放宽实机限制。
- Omega 当前平移 gain=[0.25,0.25,0.25]；旋转 gain=[1,0,1]；map/sign、wrist-joint 控制、IK、近远端补偿不改。
- PMAC prog1 预缓冲从 10 改为 5 点，已进行键盘和 Omega 采集。控制器工程中的 EtherCAT 与构建文件的其他本地改动不属于本轮。
- `apps/test_axis5_response.py`：逻辑 d 经 ContinuumAxisMapper 生成单轴五次平滑往返轨迹，50 Hz、高精度等待；默认 dry-run，执行需显式 --execute 和已确认的行程余量。其他四轴目标保持。工具执行会调用既有 PLC reset/start；不是纯读取入口。Ctrl+C 请求 abort，不自动返回。默认 1 mm、0.5 mm/s；允许最大 5 mm、2.3 mm/s，不自动提速。

`outputs/` 中的原始日志、离线重放脚本、图表和 SSH 读取记录保留在本地，不混入运行代码。失败的 `outputs/omega_02_coordination/` 协调候选仅为实验，未接入 driver：其最大滤波目标位置误差从 5.90 增至 16.79 mm，并出现电机目标速度尖峰，搁置该方案，不继续围绕它重构上层。

### 已有第五轴实测

| 峰值命令速度 | 行程 | 采样最大跟随误差 | 最终编码器返回误差 |
| --- | --- | --- | --- |
| 0.5 mm/s | 1 mm | 0.0057 mm | 0 counts |
| 1 mm/s | 5 mm | 0.0116 mm | 0 counts |
| 2 mm/s | 5 mm | 0.0177 mm | 0 counts |
| 2.3 mm/s | 5 mm | 0.0254 mm | 0 counts |

记录为 outputs/axis5_response_01.csv 至 04.csv。跟随误差是 PLC 读取的 abs(DesPos-ActPos) 整数快照，约 10 Hz；可能漏掉瞬时峰值，不是外部机械定位精度。档位的加速度不同，不能视为严格仅改变速度的试验。用户实测平顺；每档单次往返不代表长期或不同负载验收。负载、额定能力和型号仍未知。用户确认总行程约 50 cm、当时两侧余量均大于约 5 cm；相对编码器，不自动回零或设零。

SSH 只读核对过：Axis5_MaxStep=3000、Axis5_MaxVel=400、Motor[5].MaxSpeed=32、InvAmax/InvDmax/InvJmax=10/10/50、MinPos=MaxPos=0、Coord[1].SegMoveTime=0、LHDistance=0。参数适用模式不同，不能把 MaxSpeed 当成 PVT 通用保护或硬件极限。当前步长对应约 2.389 mm/s；Python/PLC 原有截断暂时保留。

### 下一步顺序

1. 固定上层、50 Hz 与 5 点缓冲，先锚定第五轴可用速度、加减速、停止/反向和跟随误差范围。
2. 新一轮开始前读取并保存当时的第五轴运行参数，明确恢复值；不因已授权读取就自动写参数。
3. 准备 3 mm/s 小行程对照，先离线确定轨迹、加速度及停止标准。当前工具封顶 2.3 mm/s，且 3000 counts/段不足以支持 3 mm/s，不能仅修改 CLI 上限就运行。需要单独准备、核对必要的临时步长变更及回退；不放宽跟随误差报警来掩盖问题。
4. 需要判断伺服参数时再采集足够分辨率的内部指令/反馈与负载，按证据调整增益/前馈，避免与轨迹约束同时大改。
5. 第五轴稳定后依次验证其他轴、五轴联动，最后匹配 IK 轨迹与 Omega 输入。不把“未报警”当作性能上限判据。

### 配置与文档边界

AGENTS.md 中的 xzy 坐标说明是历史说明，与当前 YAML 的 zxy 不一致；以已核对的配置和代码为依据，不据旧说明更改映射。新版 driver 要求 execution.insertion_max_speed_m_s，历史配置仍可用于离线读取，但实机迁移须显式补全。

根目录 teleop_*.csv 及 outputs/ 是本地证据，忽略规则只影响 Git，不删除文件。两仓库提交应仅包含本轮代码，不包含无关 EtherCAT/生成文件；不得将调试检查点命名为整体硬件验收完成。

配套 PMAC 源码检查点：`44950f3`（仅 prog1 的 5 点预缓冲）。提交本身不执行下载，控制器状态需另行核对。

### 检查点软件验证

2026-10-09，显式选择的离线回归共 160 passed、2 deselected，覆盖协议与传输、driver 生命周期、接口配置、等待器、诊断、第五轴轨迹、计算频率工具、Omega 映射、关节电机模型及 IK 边界。未执行全目录 pytest，也未启动硬件。两项未纳入的是既有 Windows SIGTERM 退出码断言测试（dry_run_holds_motor_targets_and_exits_on_sigterm），不计为通过。

Omega 平移测试改为显式参数化 0.5/0.25 增益，同时验证 0.25 增益下 1 mm 输入落入 0.3 mm 输出死区；没有为适应旧断言而恢复运行配置。Git diff --check 通过。回退代码可分别 revert 两仓库检查点提交；本地 Git 回退不会自动改变控制器已下载程序。

## 2026-10-09：限制职责分离的第一步

以下保留当时的设计思路；其中协调约束属于后续研究，不是当前待实施方案。当前工作顺序以上面的“下一步顺序”为准。

`robot_interface.yaml` 新增 `execution.insertion_max_speed_m_s`，仅定义逻辑插入轴 d 的速度预算，不再借用 `command.max_speed_m_s[2]`（WORLD Z）。当前显式值 0.01 m/s 保留原主配置的上位机预算，不代表第五轴已经验证能达到该速度。实机预算仍取该速度换算步长与控制器回传 `Axis5_MaxStep` 的较小值：50 Hz、62781744 counts/m、PLC 3000 counts/段时，有效上限仍是 3000 counts/段，约 2.389 mm/s。dry-run 不读取 PLC，只有上位机预算，不冒充实机保护值。

历史 YAML 仍可由配置加载器读取以供离线分析，但新 driver 在连接硬件前要求显式填写该执行参数，不根据 WORLD Z 静默推断。使用其他接口配置启动时需逐一核对并迁移。

开启 latency CSV 后增加 `insertion_step_limit_counts`、`insertion_requested_step_counts`、`insertion_applied_step_counts`、`insertion_step_clipped`。前两种步长分别是截断前请求和取整后实际目标增量；截断标志表示请求幅值超过当前预算。若显式锁定插入轴，请求值是锁定处理后的请求。没有额外硬件读取。

本步骤尚未改变单轴截断行为，也没有解决动态追赶。下一阶段的协调约束应位于 IK 候选目标与 PVT 提交之间，在提交任何轴之前验证完整五轴轨迹，并只将最终接受目标作为 warm-start。不能只把各轴最大增量归一化后就宣称保持笛卡尔路径；需离线验证位置与方向误差、每段插补峰值速度/加速度、hold/反向和不可达目标。各轴能力未统一前，不删除 PLC 最终保护，不修改 Omega 映射。

现象：键盘与 Omega 都表现为起停偏晚、过程顺畅。优先定位共用的滤波、提交等待和轨迹缓冲；不预先归因于电机增益。首轮保持 50 Hz 和已有参数。

## 采集

给原先确认可用的 driver 启动命令追加：

```text
--latency-csv outputs/latency_keyboard_01.csv
```

这只是追加参数，不是完整启动命令。保留原来的环境、连接参数和执行选择，不自动添加 `--execute`。driver 的实机启动本来会复位 PLC 并启动程序；只在用户安排的下一次实机测试中启用记录。不要启动第二个直连 PMAC 的程序。文件必须不存在，避免覆盖历史证据。

使用既有键盘工具，在用户已确认安全的范围内：静止约 2 秒，单方向小幅移动并停止，等待停稳，再反向；重复 3 次。不要直接使用离线 benchmark 的压力目标。记录使用哪个键、哪根轴、停止指的是松键还是显式 hold。首轮不提高反馈频率、不改变滤波、速度或缓冲。

正常 Ctrl+C 结束会关闭日志。每 50 帧刷新一次，强制终止可能丢失尾部数据。日志是同步缓冲写入，可能扰动周期；`previous_log_ms` 单独记录上一帧日志耗时（含定期刷新），`work_before_log_ms` 不包含当前日志写入和 sleep。默认不开启，无额外 PMAC 请求。

## 分析

```powershell
uv run --no-project --with numpy --with matplotlib python -B apps/analyze_latency.py outputs/latency_keyboard_01.csv --out outputs/latency_keyboard_01_analysis
```

输出：

- timing.png：实际周期间隔、IK+映射耗时、PVT 提交耗时、新反馈读取耗时、软件队列与反馈年龄。
- commands.png：最近接受的原始 WORLD 目标与实际滤波输出，平移 mm、旋转 deg。hold 后 raw 仍是最后接受的原始目标，必须结合 holding/received_kind 理解，不代表继续追赶。
- axes.png：五根物理轴的已提交目标与新编码器反馈。反馈只画真正读取的样本，横向误差条表示主机请求起止区间，不能称为准确的控制器采样时刻。
- summary.json：各耗时 P50/P95/P99/最大值。

dry-run 不填真实反馈、队列和提交耗时，不用目标冒充实测编码器。主机时间均为相对本次循环启动的 monotonic 秒。未收到新反馈时保留上一次请求区间，feedback_new=false，不能当成新采样。

queued_at_ack 是成功确认时已有状态响应中的软件队列点数，不增加 Modbus 读取。它不包含 PMAC 内部所有插补缓冲，不保证与 token 在控制器端原子采样；不能将 queued×20ms 当成精确总延迟。accepted_token 只证明入队，不能证明到达。

## 根据证据定位

1. raw→applied 明显追赶：优先看滤波/限速。当前平移限速 10 mm/s，目标位置差会产生真实的追赶时间，不是纯固定延迟。
2. 周期明显大于 20 ms：检查 submit_ms、feedback_ms、ik_mapping_ms 与 previous_log_ms 谁占时间。
3. 提交及时，所有物理轴反馈整体晚：核对控制器已下载版本、预缓冲与实际排队。10 点表示约 200 ms 的轨迹量，不直接等于固定实测延迟。
4. 需要判断伺服时，在后续控制器采集中比较同期插补位置指令与编码器。当前日志没有插补指令，不能把主机目标减反馈命名为伺服跟随误差。

默认反馈只有 10 Hz，首轮只能定位较粗的延迟。亚百毫秒延迟、控制器缓冲与伺服的分离，需后续更合适的控制器采样；不能从插值后的低频反馈图声称精确延迟。跨进程的发送端时间戳本轮未用于延迟计算，未测键盘输入到 driver 接收的完整端到端时间。

## 第二轮：协议内部计时（latency_schema=2）

仍使用 `--latency-csv`，在新启动的 driver 中自动开启协议计时，无需额外选项。已运行的进程不会因磁盘代码更新自动启用；不要另开第二个 driver。下一次由用户正常结束旧会话、启动新会话时，换用 `outputs/latency_keyboard_02.csv`，保持 50 Hz、原滤波和原 PVT 缓冲参数，重复同样的小幅键盘动作。

新增列分别带 `pvt_` 和 `snapshot_` 前缀：

| 字段 | 含义 |
| --- | --- |
| total_ms | 持有 mailbox 锁后的事务耗时，不含拿锁等待 |
| read_ms / write_ms | Modbus 读/写调用累计墙钟耗时，包含库内部和系统调度，不是纯网络 RTT |
| reads / writes | 调用次数，包括失败调用 |
| polls | wait 中执行条件检查的次数，不等于所有状态读取次数 |
| sleeps | wait 显式休眠次数 |
| sleep_requested_ms / sleep_ms | 请求休眠累计值 / 实际休眠累计耗时 |
| success | 事务是否正常完成 |

只在新的反馈请求周期填写 snapshot 数据，其余为空，避免将缓存统计重复计算。没有新增控制器访问或重试，也不调整 sleep、超时、定时器、优先级或控制频率。失败事务在协议对象中保留 success=false 并清理活动计时上下文；现有 driver 异常路径不会输出该未完成周期的 CSV 行，所以 CSV 不构成完整故障事务审计。

新增 `previous_cycle_sleep_requested_ms` / `previous_cycle_sleep_actual_ms` 对应上一周期末的调度休眠，应与本行 period_ms 对齐看。第一行两者为 0，没有前一周期。日志头保存 Python 版本、轮询间隔和 schema。

新版分析器兼容旧日志，并为新日志生成 protocol_timing.png，分别显示提交、反馈事务和循环末休眠。将每周期 read_ms + write_ms + sleep_ms 与该事务 total_ms 比较，剩余部分为 Python 处理、测量开销等；不能将各项的 P99 分别相加后当成总 P99。

判断：若 read/write 较短、sleep_requested=1 ms 而 sleep_ms 接近 15 ms，优先处理显式休眠唤醒；若耗时主要位于读写调用，先继续定位 Modbus/控制器响应，不能仅凭独立 sleep 探针就改等待方式。供点恢复稳定后再评估减小缓冲。

## 当前验证状态

已通过离线记录、滤波/hold、禁止 dry-run 创建硬件控制器、旧文件拒绝覆盖、ACK 状态复用与错误后清缓存检查。协议/生命周期组合测试中 34 项通过，两个原有子进程 SIGTERM 测试在 Windows 的退出码断言失败（运动保持断言已通过）；没有把这两个失败隐藏为全面通过。未上机采集，尚未确定真实故障根因。

使用 HEAD 未修改的 driver 脚本再次执行这两个子进程用例，同样在退出码断言失败，确认并非新增日志逻辑引入。新增分析脚本已用 dry-run 日志完成图表生成验证。

补充假控制器用例后，排除上述两个已知 Windows 退出用例的复测为 35 passed、2 deselected；假控制器验证了提交耗时、已有 ACK 队列状态和反馈请求时间区间，未访问真实硬件。

第二轮计时增加确定性时钟和延迟确认测试，验证请求 1 ms/实耗 15 ms 的区分、读写累计耗时、状态隔离、失败后清理，以及开启/关闭计时的请求序列完全一致。协议与 driver 相关复测为 36 passed、2 deselected（仍排除上述已复现的 Windows 退出用例）。

## 第三轮：验证高精度等待对实机供点的影响

使用可选 `--wait-mode windows-highres`，替换协议轮询等待及主控制循环末尾等待。默认仍为 `system`。不调整 50 Hz、滤波、速度或缓冲参数；不改变全局系统定时器分辨率，不忙等。未启用的启动返回参考路径仍保留原有循环等待。创建高精度定时器失败时在连接硬件前报错，不静默回退。

先正常结束旧键盘和旧 driver，确认只有一个进程占用 mailbox。以下是用户手动进行第三轮实机测试的 Git Bash 命令，`--execute` 会执行原有硬件启动流程。

终端一：

```bash
cd /d/project/surgical_continuum_robot/robo-pmac-driver
uv run python apps/continuum_driver_server.py \
  --interface-config config/robot_interface.yaml \
  --wait-mode windows-highres \
  --latency-csv outputs/latency_keyboard_03.csv \
  --execute
```

确认启动输出 `Wait mode: windows-highres`。日志文件必须不存在；已有文件时换新文件名，不删除旧证据。

终端二：

```bash
cd /d/project/surgical_continuum_robot/robo-pmac-driver
uv run apps/teleop_pvt_keyboard_pose.py
```

静止约 3 秒，在此前已确认安全的小范围内分别做 X（A/D）、Z（W/S）正反移动，每次松键后等待约 3 秒，重复约 3 组。尽量复现第二轮的动作和幅度，不同时按多个方向，总计约 30–45 秒。松键停止增加目标，不等于立即 hold。记录主观起停迟滞有无变化。

正常结束时先在键盘窗口按 Esc 或 Ctrl+C；该工具会执行约 5 秒返回启动位置，等待其完成，再在 driver 窗口 Ctrl+C 关闭日志。这是正常退出流程，不是急停。

采集后比较第二、三轮：请求/实际休眠、周期分布与超期比例、PVT 提交和反馈读取耗时、队列变化、目标/反馈粗略滞后。单轮动作和系统负载未完全匹配时不能作严格因果量化。若供点变稳定但仍有迟滞，再检查控制器缓冲及同期插补指令/编码器反馈。

离线验证：46 passed、2 deselected，排除上文已复现的 Windows SIGTERM 退出用例。2026-10-09 使用项目 Python 3.10.20 对生产等待实现进行每模式 100 次、请求 1 ms 的无硬件抽测：system 中位数 15.514 ms、P95 17.625 ms；windows-highres 中位数 1.714 ms、P95 2.082 ms，原始摘要位于 `outputs/highres_backend_20261009.json`。这些结果不表示实机延迟已改善，也不是实时性保证。

需要恢复时改用 `--wait-mode system` 并选新的日志文件名。
