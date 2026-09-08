# 2026-09-08 离线验证记录

验证范围为 `feat/igh-offline` 的环境与离线基座。本记录不表示真实 EtherCAT 总线、电机运动或完整 PMAC 替代已验收。

## 环境与构建

| 项目 | 结果 |
| --- | --- |
| 系统 | Ubuntu 24.04.4 LTS，x86_64 |
| 内核 | `7.0.0-31-generic`，`CONFIG_PREEMPT_RT` 未启用 |
| 编译器 / CMake / Ninja | GCC/G++ 13.3.0 / 3.28.3 / 1.11.1 |
| IgH | 1.6.12，固定提交 `2140c102058bfbabbe0677153850c51c9382343d` |
| RtIPC | 1.0.4，固定提交 `7a9a0a5dc9edeff6af45623dd8d3f6d4c374657c`，加两项本地资源释放补丁 |
| 真实库、模拟库、CLI | 本地安装并运行验证通过 |
| 内核模块 | `ec_master.ko` 与 `ec_generic.ko` 编译通过 |
| 模块 vermagic | `7.0.0-31-generic SMP preempt mod_unload modversions` |
| 有线网口 | `enp0s31f6` / e1000e，carrier=0 |
| 现场状态 | 无主站模块加载，无 `/dev/EtherCAT*`，无实机通信 |

CLI 输出为 `IgH EtherCAT master 1.6.12 unknown`；其中 `unknown` 是该浅克隆构建的版本描述后缀，实际源码提交已固定并在准备脚本中校验。

## 测试结果

运行 `bash scripts/prepare_igh.sh` 和 `bash scripts/test_offline.sh`，5 个 CTest 分组全部通过：

| 分组 | 结果与范围 |
| --- | --- |
| `core` | 19/19：状态字、控制字、PDO 编码、PVT 极值与边界、队列、心跳、异常门控 |
| `real_library_abi` | 真实 `libethercat` API 1.6 与头文件一致；未请求主站 |
| `fake_master_pdo` | 官方模拟库完成五轴配置、120 字节过程映像、21 个合成轨迹采样和退出 |
| `pmac_contract` | ENI、PMAC IO 配置及模式设置交叉检查通过 |
| `importer_regression` | 9/9：错误标识、偏移、填充、从站顺序、DC、SDO 和模式均被拒绝 |

本项目 C++ 检查启用 AddressSanitizer、UndefinedBehaviorSanitizer 和泄漏检查，最终测试无报告。最初发现的 RtIPC 数组释放不匹配和目录句柄泄漏已分别保存为 `patches/rtipc-array-delete.patch`、`patches/rtipc-close-directory.patch`。

原始输出在本机 `build/offline/results.xml`、`build/offline/Testing/Temporary/LastTest.log`、`build/offline/fake-smoke.log`、`build/host-report.json`。这些构建产物被 Git 忽略；复现命令见上级 README。

## 普通调度下的空循环计时

运行 `build/offline/host_timing_probe 3000`。测试时为普通桌面环境，IgH 构建同时进行，调度策略为 `SCHED_OTHER`，无绑核和实时优先级设置。

| 量 | 结果 |
| --- | --- |
| 目标周期 / 样本数 | 1 ms / 3000 |
| 唤醒延迟中位数 | 54.282 μs |
| 唤醒延迟 P99 | 135.409 μs |
| 唤醒延迟最大值 | 268.373 μs |
| 跳过的完整周期 | 0 |

这只是约 3 秒的空循环基线，没有 EtherCAT 收发、IK、插补负载或真实 DC。不能用它推断最坏延迟、长时间稳定性或实机控制合格；实时内核与完整控制线程仍需单独验证。

## 前次修复的分支隔离

前次 PMAC/PVT/IK 修复重新运行 60 项离线回归测试，全部通过，然后提交到独立分支：

| 仓库 | 分支 | 提交 |
| --- | --- | --- |
| robo-pmac-driver | `fix/pmac-pvt-ik` | `4475a5e` |
| surgical_continuum_robot_pmac | `fix/pmac-pvt-ik` | `a32e2bd` |
| surgical_continuum_robot | `fix/pmac-pvt-ik` | `f9ba7f1` |

IgH 分支从驱动原 `dev-pvt` 的 `0dc4323` 建立，不包含上述修复提交。驱动修复分支保留在相邻 `robo-pmac-driver-pmac-fixes` 工作目录，仿真原有的未跟踪实验和结果未纳入提交。所有提交仅在本地，未推送。
