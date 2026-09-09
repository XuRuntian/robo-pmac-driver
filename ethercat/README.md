# IgH 环境与离线测试

当前是迁移的第一阶段：构建真实 IgH 库、命令行工具和内核模块，建立五轴配置与离线验证程序。**尚未提供实机运动入口，也未移植完整回零、停止、线程调度或上层 IK 通信后端。**

最新现场结果：2026-09-09 14:38 用户纠正编码器反馈插接后，单台 `603F=0`、`6041=0x0240`（未使能），连续 10 秒无 Fault，位置反馈恢复。本次未发送复位或修改 PDO；PDO 分配仍需后续处理。见 [反馈纠正后复查](docs/encoder_reconnected_2026-09-09.md)。

本机的构建、测试和计时结果见 [2026-09-08 离线验证记录](docs/offline_validation_2026-09-08.md)。

2026-09-09 已完成真实内核模块的未接线加载测试：主站与通用网卡模块正常，指定网卡成功附着，结束后已卸载。见 [未接线主站加载验证](docs/unconnected_validation_2026-09-09.md)。

同日接线后已识别 3 台 Diamond 驱动器并读取诊断 SDO，数量与用户说明相符。三台均报 `0x738A`，PDO 分配表还包含空索引；尚未验证运行状态或运动。见 [首次接线检查](docs/connected_validation_2026-09-09.md)。

随后改为单台并接好电机/编码器，尝试了一次禁止使能的 SDO 故障复位；驱动器接受写入，但复查仍报 `0x738A`，观察中还出现邮箱响应不匹配。见 [单台复位记录与限制](docs/single_drive_reset_2026-09-09.md)。

后续只读对照复现了邮箱错误；等待自动 SDO 字典读取完成后，连续 10 秒的 55 次上传全部成功。诊断脚本已加入这一等待，PDO 未改动、`0x738A` 仍存在。见 [PDO、邮箱与编码器故障定位](docs/diagnostic_isolation_2026-09-09.md)。

## 分支与工作目录

- `feat/igh-offline`：本目录的环境、配置提取、C++ 基础组件和离线测试。从原 `dev-pvt` 独立建立。
- `fix/pmac-pvt-ik`：之前的 Python PMAC/PVT/IK 修复。当前机器另有工作目录 `../robo-pmac-driver-pmac-fixes`。
- 配套 `surgical_continuum_robot_pmac` 和仿真仓库 `surgical_continuum_robot` 的前次修复，也分别保存在各自的 `fix/pmac-pvt-ik` 分支。

IgH 分支不包含前次 PMAC 修复提交。后续连接上层 IK 时，需要明确合入或移植哪些通用修复。当前离线测试不调用原 PMAC 控制入口。

## 准备与复现

在 Ubuntu 上，系统构建依赖为：

```bash
sudo apt-get install build-essential autoconf automake libtool pkg-config \
  cmake ninja-build git python3 libyaml-dev zlib1g-dev "linux-headers-$(uname -r)"
```

进入驱动仓库的 `ethercat/` 后：

```bash
bash scripts/prepare_igh.sh
bash scripts/test_offline.sh
build/offline/host_timing_probe 3000 > build/host-timing.json
```

`prepare_igh.sh` 首次需要联网获取源码；源码和系统依赖已有后，构建与测试可以断网运行。IgH 与 RtIPC 的版本固定在 [sources.json](sources.json)，不会自动跟随远端分支更新。

构建、安装均在项目内的 `build/` 完成，脚本不需要 root。`build/igh-install/bin/ethercat version` 可验证本地 CLI。脚本没有 `modules_install`、`modprobe`、服务启动、网口绑定或电机使能操作。系统内核升级后，应重新执行准备脚本；主机检查会拒绝将旧模块算作当前内核已就绪。

主要产物：

| 路径 | 内容 |
| --- | --- |
| `build/igh-install/` | `libethercat`、`libfakeethercat`、`librtipc`、头文件和 CLI |
| `build/igh-source/master/ec_master.ko` | 当前目标内核的主站模块，未加载 |
| `build/igh-source/devices/ec_generic.ko` | 通用网卡模块，未加载 |
| `build/offline/` | C++ 程序、CTest 日志和 `results.xml` |
| `build/host-report.json` | 只读环境检查，包括内核、网口和编译状态 |
| `build/host-timing.json` | 普通调度下 1 ms 空循环唤醒测量 |

## 五轴配置来源

[config/pmac_drives.json](config/pmac_drives.json) 从相邻控制器仓库的 `PowerPMAC/Configuration/eni.xml` 提取，并与 `ECATConfig.cfg`、运行模式设置交叉校验。它是离线迁移依据，`hardware_validated` 为 `false`。

| 项目 | 工程中的值 |
| --- | --- |
| 从站位置 | `0, 1, 2, 3, 4`，由 ENI AutoIncAddr 得出 |
| 别名 | 全部为 `0` |
| Vendor / Product / Revision | `0x418108 / 0x9252 / 1` |
| 模式 | `0x6060 = 8`，CSP |
| SM2 / RxPDO | `0x1600`，10 字节，包含末尾 1 字节填充 |
| SM3 / TxPDO | `0x1a00`，14 字节，数字输入前有 1 字节填充 |
| 应用周期 / Sync0 | 1 ms / 1 ms |
| DC AssignActivate | ESC `0x0980` 写入 `00 07`，小端值 `0x0700` |
| Sync1 寄存器 | ESC `0x09a4` 写入 `500000 ns` |
| DC 参考时钟 | 第一个从站 |

`sync1_register_ns` 保存的是 ESC 寄存器原值；不能直接把它解释成独立的 0.5 ms Sync1 周期。IgH 对 Sync1 的语义、驱动器 ESI 和真实同步表现，需要下一阶段核对。

PMAC 的 `Slave[].Position` 在该文件中全为零，不能直接作为 IgH 的五个位置。PMAC 过程映像的字节偏移也不能直接复用；适配代码通过 `ecrt_slave_config_reg_pdo_entry()` 获取偏移。这里只把明确映射到 SM2/SM3 的 PDO 转换为 IgH 配置，不复制 ENI 中主站底层初始化帧。

校验或更新配置：

```bash
python3 scripts/import_pmac.py --check
# PMAC 配置确实改变时，重新提取并审查差异：
python3 scripts/import_pmac.py
git diff -- config/pmac_drives.json
```

出现从站数量、标识、PDO 布局、模式、DC 参数不一致或额外启动 SDO 时，提取器会报错。首次构建需要相邻 PMAC 仓库；也可用 `--pmac /path/to/PowerPMAC` 指定提取来源。

## 离线检查覆盖范围

- 真实 `libethercat` 与头文件的 API 版本检查：只调用 `ecrt_version_magic()`，不请求主站。
- 官方 `libfakeethercat`：创建五轴配置、注册 PDO、分配 120 字节模拟过程映像、运行 21 个合成 PVT 采样点并释放资源。全程控制字为禁用状态；反馈由测试注入，不代表电机跟随性能。
- C++ 核心：状态字按位掩码解码、显式故障复位、带符号与非对齐 PDO 读写、五轴三次 Hermite 插补、整段位置/速度/加速度极值检查、队列满/顺序/回绕、反馈和心跳过期、掉线/WKC/模式/驱动异常、队列欠载和丢周期门控。
- 配置提取：故意破坏标识、布局填充、偏移、模式、从站顺序、DC 和启动 SDO 的回归测试。
- 单台复位脚本：模拟验证只写 `6040=0/128/0`、身份/数量/状态门控、写入结果不确定时清除复位位，以及读取失败或持续故障时不重复复位。
- 字典等待：以每台的实际完成记录为条件；开始记录或部分从站完成均不放行，超时停止外部 SDO 读取。
- 本项目 C++ 测试默认启用 AddressSanitizer、UndefinedBehaviorSanitizer 和泄漏检查。

`MotionGate` 只判断是否允许继续计算目标，**没有实现物理停止**。`SegmentQueue` 是单线程离线队列；PVT 接口使用 counts/s，与原 PMAC 的 counts/ms 不同。示例里的位置、速度、加速度限值都是合成测试值，不是机器人标定值。

IgH 模拟库的 WKC/主站状态通常直接返回成功，因此上述异常通过独立门控测试注入；它不能验证真实总线丢包、DC 时序、驱动器状态转换或电机动力学。空循环计时同样不属于 EtherCAT 或实时控制验收。

## RtIPC 本地补丁

所固定的 RtIPC 版本在模拟测试退出时触发了两项检查：数组用 `delete` 释放，以及 `opendir()` 的目录句柄未关闭。`patches/` 保存对应的小补丁，准备脚本自动应用并允许重复执行；没有关闭 Sanitizer 来绕过错误。补丁仅用于本地模拟依赖，未向上游提交。

## 下一阶段

已完成临时模块加载、3 台及单台从站识别，并通过等待字典完成避免诊断邮箱并发。纠正编码器插接后 `0x738A` 已不再出现，当前没有故障复位需求。下一步以已有 PMAC ENI 为依据整理明确的 PDO 初始化，在禁止使能状态验证周期通信与 DC。上电实际位置约为 -506000555 原始 counts，必须先完成目标与实际位置对齐和单位/零点标定；随后验证急停链、停止和使能状态机，最后开展限幅单轴运动。

接线后的诊断可在 `ethercat/` 运行：

```bash
sudo python3 scripts/probe_connected.py enp0s31f6 > build/connected-probe.json
```

脚本仅供已隔离原主站的调试链路使用；拒绝已有 IgH 模块、无链路或不匹配的内核模块。它临时加载空闲主站进行发现及有界 SDO 读取，退出时卸载本次加载的模块。发现过程会初始化 EtherCAT 状态和邮箱，通常进入 PREOP；脚本没有应用 PDO 配置、SDO 写入、故障复位或使能/运动命令。退出码 0 表示诊断流程与清理成功，仍须检查 JSON 中的 `warnings`、状态字和错误码，不能视为运动就绪。

诊断脚本依赖本机 `journalctl` 的内核日志，临时以 debug level 1 加载主站，等待每台实际出现 `Fetched ... SDOs and ... entries` 后将调试级别降为 0，再开始应用 SDO 读取。45 秒内无法确认全部完成会退出；不会通过固定睡眠时间猜测字典已就绪。该措施限定于这些临时诊断入口，没有修改 IgH 内核源码或为其他应用实现通用邮箱互斥。

单台持续观察和配置读取也使用同一等待流程：

```bash
sudo python3 scripts/watch_diagnostics.py enp0s31f6 > build/watch.json
sudo python3 scripts/inspect_drive_config.py enp0s31f6 > build/drive-config.json
```

上述两个入口均无驱动器写命令；配置读取根据设备返回的对象名称、类型和 PREOP 读权限选择身份/主站类型/编码器相关参数，最多读取 64 项。

完整替换 PMAC 还需要实现实时轨迹流、启动时实际位置对齐、零点/单位标定、回零、跟随误差与限位保护，以及通信中断时的协调停止。上一分支保留的 PMAC 方案可用于对照。

参考：[IgH 官方主站](https://etherlab.org/en_GB/ethercat)、[IgH 1.6 API](https://docs.etherlab.org/ethercat/1.6/doxygen/group__ApplicationInterface.html)、[官方模拟库说明](https://gitlab.com/etherlab.org/ethercat/-/blob/2140c102058bfbabbe0677153850c51c9382343d/fake_lib/README.md)、[CiA402 状态字编码参考](https://drives.novantamotion.com/summit/0x6041-status-word-doc)。驱动器专用位含义以实际设备手册为准。
