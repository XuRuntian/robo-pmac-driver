# Python 控制计算频率验证

## 目标与阶段

先测现有 Python 实现的计算能力和截止时间表现，再在当前 50 Hz 下定位实机滞后，最后决定是否提高实机频率。本工具不优化 IK，不改变任何生产配置，也不连接 Omega、ZMQ 或 PMAC。不存在 `--execute` 参数。

1. 短测验证计时、输入生成、误差统计与离线隔离。
2. 无等待扫频，筛查计算瓶颈；接近通过/失败分界后缩小频率间隔。
3. 对候选频率做 `--paced` 固定截止时间复测、增加时长及重复次数。
4. 后续单独授权硬件测试，补上 Modbus/反馈/PLC 排队和真实伺服测量。本工具结果不能替代这一步。

## 两种工作负载

- `ik`：根据已知关节轨迹生成 FK 位置与工具轴方向，再交给现有 IK 连续求解，保留 solver warm-start。FK 生成输入不计入耗时。它验证同一数学模型的数值一致性，不证明模型与机械一致。
- `pipeline`：WORLD 字段校验、现有滤波与坐标变换、`ContinuumPVTMapper.build_command`、软件第五轴步长限制、接受脉冲反算与 warm-start、命令 FK。使用非零合成基准，通过 `ContinuumAxisMapper` 映射，未使用真实编码器参考。

`pipeline` 是驱动的**计算子集**，未包括 ZMQ 收发/序列化、watchdog、诊断输出、反馈状态重建、Modbus/ACK 等。计算耗时不能称为完整驱动周期。下位机 Axis5_MaxStep/MaxVel 生效值未知，离线仅复现驱动软件限速，模拟立即接受提交。

现有 AGENTS.md、debug_full_chain.py 和部分配置注释存在历史坐标描述。本工具读取实际 robot_interface.yaml 的 map/sign，不复制历史硬编码。生产配置不作改动。

## 场景与可比性

固定 0.2 Hz 连续输入，按目标采样频率重采样，不把相同步长在高频下重复执行。默认时长 10 秒，即两个完整周期；默认重复三次，随机化组合顺序（固定随机种子）。每组合先在独立状态预热三帧，再重建状态计时，计时样本包含首帧。预热不是冷进程启动性能测试。

| 场景 | 初值 d/theta_a/phi_a/theta_c/phi_c (m/rad) | 目的 |
| --- | --- | --- |
| normal | 0.12 / 0.5 / 0.4 / 0.3 / -0.5 | 平滑连续运动 |
| near_straight | 0.12 / 0.001 / 0.4 / 0.001 / -0.5 | 近直线状态 |
| boundary | 0.254 / 3.10 / 0.4 / 1.53 / -0.5 | 插入与弯曲边界附近 |
| unreachable | 与 normal 相同 | IK：FK 目标加 [0.5,0.5,0.5] m；pipeline：大 WORLD 目标经过实际限幅/滤波 |

初值用于默认 continuum.yaml，不是实机标定。IK 的前三组目标由边界内关节生成；pipeline 的 WORLD 目标不保证可达，尤其边界场景。`pipeline/unreachable` 的名字表示压力输入意图，经过限幅和滤波后不保证每帧都不可达。必须分组报告收敛率，不能把压力输入的不收敛简单解释为性能失败。

WORLD 常规输入平移为 [0.002 sin(wt), 0.001(1-cos(wt)), 0.002 sin(wt)] m，旋转为 [0.03 sin(wt),0,0.02(1-cos(wt))] rad，w=2*pi*0.2。IK 轨迹参数见工具中的 `reachable_goal`，元数据保存源文件哈希。平滑系数按周期生效，因此升频会改变滤波时间响应；每档误差相对当档实际滤波目标计算，不能宣称滤波后的轨迹完全相同。

## 指标与判断

- work_ms：计算子集的墙钟耗时，不含等待、离线输入生成与事后误差分析；包含计时探针的小量开销。
- P50/P95/P99/观测最大值、计算超期比例、最长连续计算超期次数。
- `--paced`：以固定 epoch+n/f 调度，用普通 sleep，不修改 Windows 定时器、不忙等；记录实际周期间隔与完成截止时间超期。落后不重置时间轴，因此可以暴露积累的延迟。样本保存开销影响随后调度，属于本测试运行器的开销。
- 现有 driver 在超期时重置下一周期时间，本运行器不重置。因此 paced 是严格周期调度压力测试，不是 driver 原样回放；不能将它的累积延迟直接当成 driver 或电机延迟。无等待与 paced 运行也可能因 CPU 唤醒/调度等因素产生不同耗时分布，须分开报告。
- FK 位置误差（mm）、真实工具轴夹角（deg）、求解器收敛与限位尝试标志；pipeline 同时记录脉冲限幅/量化后的模型误差。这些均非末端实测误差。
- `compute_pass`：该组观测中没有计算耗时超过周期；`reserve_30pct_pass`：观测最大值不超过周期的 70%。30% 为本次工程筛选余量，不是行业标准，也不保证剩余时间足够通信。
- 候选频率必须同时考虑正常可达任务精度、困难任务表现，以及 paced 调度结果。压力场景另列。只报告测试档位中的最高通过值；最高档通过时写“上限尚未找到”，不能把最高测试档当绝对上限。有限测试不能给出 Windows/Python 的硬实时保证。

当前本地 PLC 要求 PVT 段时长 >5 ms；一周期一段且段时长=周期时，200 Hz 及以上不兼容当前实现。软件仍可离线测更高档位以找到计算瓶颈，不表示授权修改协议或提高实机频率。

## 运行（PowerShell）

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
uv run --no-project --with pytest --with numpy --with pyyaml python -B -m pytest -q -p no:cacheprovider tests/test_control_frequency_benchmark.py

uv run --no-project --with numpy --with pyyaml --with matplotlib python -B apps/benchmark_control_frequency.py --frequencies 50 75 100 125 150 200 300 500 --seconds 10 --repeats 3 --out outputs/frequency_sweep

# 根据扫频结果选择档位；这里仅举例，不是预先给出的建议频率。
uv run --no-project --with numpy --with pyyaml --with matplotlib python -B apps/benchmark_control_frequency.py --frequencies 75 100 --seconds 30 --repeats 3 --paced --out outputs/frequency_paced
```

已有输出目录会拒绝覆盖。输出包含 samples.csv、summary.csv、metadata.json 和图表。元数据保存环境、参数、Git 状态、源文件哈希、两份实际 YAML、合成初值和轴换算参数，不保存 PMAC 凭据。部署环境不同必须重新测量；正式测量记录后台负载，并避免与其他测试并发。

首次实现的短测结果见 [2026-10-08 测量记录](control_frequency_results_20261008.md)。
