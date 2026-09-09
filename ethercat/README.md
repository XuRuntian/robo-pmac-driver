# IgH 环境、离线测试与单轴通信调试

已构建真实 IgH 库、命令行工具和内核模块，建立五轴配置、离线验证以及保持失能的单轴实机周期测试入口。**尚未提供实机运动入口，也未移植完整回零、停止、实时线程调度或上层 IK 通信后端。**

最新现场结果：2026-09-09 用户提供的 ESI 与实机匹配。单台已进入 OP，首轮 10 秒 WKC 全为 3/3；第二轮出现 `0x001A` 同步错误，PDO 数量写入与回读也不一致，**稳定性验收未通过**。全程未使能；15:02 最后只读复查 `6040=0 / 6041=0240 / 603F=0`，模块已卸载。见 [ESI 与周期通信记录](docs/esi_cyclic_validation_2026-09-09.md)。

此前 14:38 用户纠正编码器插接后，`0x738A` 消失，位置反馈恢复。见 [反馈纠正后复查](docs/encoder_reconnected_2026-09-09.md)。

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

`sync1_register_ns` 保存的是 ESC 寄存器原值；固定版本 IgH 会将该 API 参数直接写入 `09A4`，不能把它解释成独立的 0.5 ms Sync1 周期。新 ESI 的默认因子按 IgH 规则换算为 `09A4=0`，与 PMAC 不同；本次失能测试保留 PMAC 的 500000 ns，后续需要对照验证相位和调度。

用户提供的 ESI 保存在 `config/esi/`，`scripts/check_esi.py --check` 校验身份、PDO 类型/方向、重映射能力、启动 SDO 和 DC 模板，结果为 [esi_review.json](config/esi_review.json)。ESI 的默认 2 ms 启动周期已明确覆盖为 1 ms：适配器新增 `60C2:1=1, 60C2:2=-3`，与主站/Sync0 一致；未下载电机或编码器对象默认值。

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
- ESI 兼容性：拒绝身份、映射方向、类型、位宽、启动命令和 DC 因子的未审查变化；保持原 ENI 提取数据和新的启动选择各自可追溯。
- 单轴失能入口：数量/身份/故障/现有使能状态门控；输出始终禁用并保持带符号反馈目标；周期失败时跳过收尾外部 SDO，避免与自动重配置冲突。
- 本项目 C++ 测试默认启用 AddressSanitizer、UndefinedBehaviorSanitizer 和泄漏检查。

`MotionGate` 只判断是否允许继续计算目标，**没有实现物理停止**。`SegmentQueue` 是单线程离线队列；PVT 接口使用 counts/s，与原 PMAC 的 counts/ms 不同。示例里的位置、速度、加速度限值都是合成测试值，不是机器人标定值。

IgH 模拟库的 WKC/主站状态通常直接返回成功，因此上述异常通过独立门控测试注入；它不能验证真实总线丢包、DC 时序、驱动器状态转换或电机动力学。空循环计时同样不属于 EtherCAT 或实时控制验收。

## RtIPC 本地补丁

所固定的 RtIPC 版本在模拟测试退出时触发了两项检查：数组用 `delete` 释放，以及 `opendir()` 的目录句柄未关闭。`patches/` 保存对应的小补丁，准备脚本自动应用并允许重复执行；没有关闭 Sanitizer 来绕过错误。补丁仅用于本地模拟依赖，未向上游提交。

## 下一阶段

已完成临时模块加载、3 台及单台从站识别、ESI 核对及初次失能 OP 周期通信。纠正编码器插接后 `0x738A` 已不再出现。下一步解决 `0x001A` 同步错误、调度延迟及 PDO 数量回读差异，在失能状态完成稳定性验证。上电实际位置约为 -506000555 原始 counts；周期测试已从实时反馈初始化目标，仍须完成单位/零点标定、急停链、停止和使能状态机，最后开展限幅单轴运动。

单轴周期测试在 `ethercat/` 运行 `sudo python3 scripts/probe_disabled.py enp0s31f6`。这个入口会写入易失的 PDO/CSP/插补/DC 通信配置，所有输出控制字始终为零；最多等待 20 秒进入 OP，随后观察 10 秒。它仍会因已知数量回读差异或同步错误返回失败，不能把一次 OP 当作验收通过。详情和现有限制见上述周期通信记录。

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
