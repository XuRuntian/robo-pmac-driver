# 为什么请求休眠时间与实际时间不一致

## 结论与边界

当前环境的主要放大来源已定位到 Windows 普通超时等待路径的唤醒粒度：同一 Python 3.10.20 进程中，仅更换等待机制，1 ms 请求的中位实际等待从 15.57 ms 降至 1.56 ms。不是 Python 与 PMAC 时钟不一致，也不是 IK 计算慢。精确到 Windows 内核的定时器到期、线程就绪、线程真正获调度各占多少，本轮没有 ETW 调度追踪，不能进一步拆开归因。

本轮完全离线、不导入机器人模块、不联网、不连接设备，没有修改 driver、PMAC 参数、Python 环境、系统定时器分辨率、进程优先级或亲和性。只新增实验工具及报告。高精度定时器仅用于实验进程内，结束时关闭句柄；没有采用忙等。并不证明实机总迟滞已经全部解决。

## 源码和官方依据

当前项目 .venv/Scripts/python.exe 是 Python 3.10.20，Windows build 26200，计时使用 QueryPerformanceCounter。计时器的测量分辨率不等于 sleep 的唤醒精度。

[CPython v3.10.20 timemodule.c](https://github.com/python/cpython/blob/v3.10.20/Modules/timemodule.c#L1951) 的 Windows 路径将秒向上取整为毫秒；主线程正时长等待使用 WaitForSingleObjectEx 等待 SIGINT 事件超时，非主线程或零时长使用 Sleep。本实验在主线程直接比较了同类事件超时与 Windows Sleep，结果与 time.sleep 接近。没有用二进制调试断言发行版与上游源码逐字一致。

[Microsoft Sleep 文档](https://learn.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-sleep) 说明系统时钟分辨率影响等待，线程在等待结束后只是变为可运行，不保证立即执行。因此 1 ms 参数不是精确的 1 ms 唤醒承诺。

[CreateWaitableTimerExW 文档](https://learn.microsoft.com/en-us/windows/win32/api/synchapi/nf-synchapi-createwaitabletimerexw) 提供 CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 标志，适用于不能接受数毫秒到期延迟的情形；普通 waitable timer 与高精度 timer 在本实验中必须区分。

[Python 3.11 更新说明](https://docs.python.org/3.11/whatsnew/3.11.html#time) 记录 Windows sleep 改用高分辨率 waitable timer。文档中的 100 ns 分辨率不能理解为实际唤醒误差只有 100 ns。本轮未升级或实测 Python 3.11，也未验证全项目升级兼容性。

## 实验设计

工具 apps/benchmark_windows_wait.py 只依赖标准库，通过 ctypes 调用 Windows API。五种方式：time.sleep、Windows Sleep、未触发事件的超时等待、普通 waitable timer、高精度 waitable timer。只有最后一种使用 HIGH_RESOLUTION 标志。定时器用负数相对到期值（100 ns 单位）、单次触发，超时等待有上限；句柄用 finally 清理。该封装是实验探针，不是已验收的生产等待组件。

每种方式测试请求 1/5/10/20 ms，每组合 100 样本、3 次重复，固定随机种子打乱测量块顺序，每块先丢弃 3 次预热。共 6,000 次计时样本。另按 driver 相同的“每轮截止时间推进 20 ms，超期后重置”调度策略运行五种方式，各 100 周期×3 次，共 1,500 周期。周期测试仅含固定小型 CPU 运算，没有 IK、通信或模拟 PMAC。

原始数据和环境信息：outputs/windows_wait_cause_20261008/{samples,blocks,summary,periods}.csv、metadata.json、period_summary.json。后台负载没有人为固定，不属于硬实时最坏时间测量。

## 实测结果

| 方式 | 请求 1 ms：中位数 | 请求 5 ms：中位数 | 请求 20 ms：中位数 |
| --- | ---: | ---: | ---: |
| Python 3.10 time.sleep | 15.57 ms | 15.65 ms | 31.18 ms |
| Windows Sleep | 15.52 ms | 15.57 ms | 31.38 ms |
| 事件超时等待 | 15.61 ms | 15.61 ms | 31.23 ms |
| 普通 waitable timer | 15.72 ms | 15.50 ms | 31.18 ms |
| 高精度 waitable timer | 1.56 ms | 5.39 ms | 20.40 ms |

高精度方式 1 ms 请求的 P95=1.79 ms，最大=2.13 ms，仍有额外延迟。不能保证任何时刻都按时唤醒。实验进程 CPU 时间占单核墙钟比例约 0–2%，没有忙等；短块 process_time 量化较粗，0% 不代表完全不耗 CPU，也不能推导整套 driver 的 CPU 占用。

50 Hz 小负载周期实验：

| 方式 | 平均周期 | 周期间隔 P95 | 周期间隔与 20 ms 之差的绝对值 P95 |
| --- | ---: | ---: | ---: |
| Python 3.10 sleep | 20.08 ms | 31.86 ms | 11.86 ms |
| 高精度 waitable timer | 20.00 ms | 20.38 ms | 0.46 ms |

普通等待在小负载下也能达到接近 50 Hz 的平均值，因为截止时间调度会用随后较短的等待追赶，但周期间隔抖动很大。不能只看平均频率，也不能将离线的 49.99 Hz 当成实机已验证频率。

## 与第二轮实机日志的联系

第二轮记录真实协议内的 1 ms 请求，实际中位数约 10 ms；本轮连续单独休眠约 15.6 ms。两者不矛盾：等待的发起时刻、间隔中已有计算和 I/O、系统调度状态不同。约 15.6 ms 的量化现象与普通等待机制相符，但没有记录内核 tick 相位，不能把每个实机样本都精确解释为“下一 tick”。

第二轮已经直接证明显式休眠占 PVT/反馈事务累计耗时 53.5%/63.0%，本轮进一步证明改变等待机制可以显著减小该类额外延迟。尚未证明改变后实际轮询次数、通信负载、缓冲和机械响应会如何变化，必须保留下一轮实机验证。

## 下一步实施建议

在当前 Python 环境内实现可切换的高精度等待组件，覆盖协议轮询和 driver 周期末；保留默认旧路径便于 A/B。保留请求间隔、ACK/超时、50 Hz、滤波/限速和缓冲点数。接入前检查初始化失败、句柄清理、Ctrl+C/异常退出及非 Windows 回退；不把本实验包装直接当作生产实现。先离线回归，再由用户安排同动作第三轮采集。

复现（Git Bash，不连接硬件）：

```bash
.venv/Scripts/python.exe -B apps/benchmark_windows_wait.py --out outputs/windows_wait_repeat
```

输出目录必须不存在。图表 comparison.png 展示请求/实际中位数和 50 Hz 周期间隔分布。
