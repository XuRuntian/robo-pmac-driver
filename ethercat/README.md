# IgH 环境、离线测试与单轴运动调试

已构建真实 IgH 库、命令行工具和内核模块，建立五轴配置、离线验证、单轴失能通信及空载小角度实机往返入口。**完整回零、多轴协调停止、轨迹流及上层 IK 通信后端尚未完成。**

最新现场结果：2026-09-09 保留普通内核，以 2 ms 周期完成单台空载电机的 **90 counts（按编码器比例约 0.247°）往返并失能**。运动期间最大跟随差 6 counts，DC 最大 3.606 µs，无不完整 WKC 或掉 OP。结束时 `6040=0`、`6041=0x1640`（失能）、`603F=0`；临时保护参数已恢复并回读，模块已卸载。见 [首次空载运动记录](docs/single_motion_validation_2026-09-09.md)。

此前普通内核的调度/DC 优化及 1 ms 偶发反馈缺失，见 [普通内核优化记录](docs/generic_kernel_validation_2026-09-09.md)。PDO 计数固定回读已作单台固件兼容诊断，尚未修复固件，也不能将本次结果外推到五轴或带机构运行。

最初 ESI 接入及 `0x001A` 同步错误的记录见 [ESI 与周期通信记录](docs/esi_cyclic_validation_2026-09-09.md)。

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

`sync1_register_ns` 保存的是 ESC 寄存器原值；固定版本 IgH 会将该 API 参数直接写入 `09A4`，不能把它解释成独立的 0.5 ms Sync1 周期。新 ESI 的默认因子按 IgH 规则换算为 `09A4=0`，与 PMAC 不同。原五轴配置保留 500000 ns，本次单轴入口采用 ESI 的 0。

用户提供的 ESI 保存在 `config/esi/`，`scripts/check_esi.py --check` 校验身份、PDO 类型/方向、重映射能力、启动 SDO 和 DC 模板，结果为 [esi_review.json](config/esi_review.json)。五轴适配默认明确设置 `60C2:1=1, 60C2:2=-3`，与主站的 1 ms 一致；单轴入口按所选周期覆盖，运动入口固定为 2 ms。未下载电机或编码器对象默认值。

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
- 单轴运动入口：默认只验证失能 PDO 回读；显式选择才运行空载 90 counts 往返。覆盖端点未到达、异常锁存、负数参数传递、临时限值恢复及停机未确认时保留限值。
- 本项目 C++ 测试默认启用 AddressSanitizer、UndefinedBehaviorSanitizer 和泄漏检查。

`MotionGate` 只判断是否允许继续计算目标，**没有实现物理停止**。`SegmentQueue` 是单线程离线队列；PVT 接口使用 counts/s，与原 PMAC 的 counts/ms 不同。示例里的位置、速度、加速度限值都是合成测试值，不是机器人标定值。

IgH 模拟库的 WKC/主站状态通常直接返回成功，因此上述异常通过独立门控测试注入；它不能验证真实总线丢包、DC 时序、驱动器状态转换或电机动力学。空循环计时同样不属于 EtherCAT 或实时控制验收。

## RtIPC 本地补丁

所固定的 RtIPC 版本在模拟测试退出时触发了两项检查：数组用 `delete` 释放，以及 `opendir()` 的目录句柄未关闭。`patches/` 保存对应的小补丁，准备脚本自动应用并允许重复执行；没有关闭 Sanitizer 来绕过错误。补丁仅用于本地模拟依赖，未向上游提交。

## 下一阶段

已完成单台空载小幅往返及正常失能；当前 2 ms 用于单轴调试，五轴原 ENI 契约仍保留 1 ms。下一步是单轴重复性、异常停止和长时稳定性，再接入多轴/PVT/IK。位置约 -506000550 原始 counts，必须从新鲜反馈对齐目标，不能把目标零当作保持位置。

单轴周期测试在 `ethercat/` 构建和运行：

```bash
bash scripts/build_commissioning.sh
sudo python3 scripts/probe_disabled.py enp0s31f6 --cpu 2 --period-us 2000 --dc esi --seconds 30 > build/disabled-probe.json
```

`build/commissioning/` 是 Release 构建；`build/offline/` 仍保留 Sanitizer 和全部离线测试。CPU 2 对应本机选择，换机器需检查允许的 CPU。FIFO 优先级 60、内存锁定和 CPU 绑定仅作用于测试进程；没有更换内核、修改全局 IRQ/省电策略或安装常驻服务。可用 `--ordinary-scheduler`、`--dc pmac` 对照；周期支持 1000/2000/4000 us，观察时长 5–120 秒。

这个入口写入易失的 PDO/CSP/插补/DC 通信配置，所有控制字始终为零。它最多等 20 秒进入 OP，再最多等 45 秒确认 DC 误差连续 3 秒不超过 20 us，随后开始正式观察。首次 WKC/OP 异常立即结束；循环中不打印日志。进程退出码 0 仅表示观察和回读完成，仍须检查 `dc_within_diagnostic_bound`、`pdo_counts_match` 和 `warnings`，不能视为运动许可。

PDO 有效前缀和实际第一项分配必须与配置一致；发现额外非零分配或布局不同仍拒绝。失能计时脚本将“内容匹配、数量不符”分别报告为 `pdo_payload_matches=true`、`pdo_counts_match=false`。后续专项脚本已清零残留槽，并验证计数写 0/有效数量后仍返回固定值，详见首次运动记录。

`scripts/inspect_motion_setup.py` 只读实际编码器比例、跟随误差、限位及停止参数。`CommissioningMotion` 已接入独立的 `igh_motion_probe`，旧 `igh_disabled_probe` 仍不包含使能入口。

仅用于本次相同固件、单台空载、人员在场可立即断电的装置：

```bash
# 诊断固定计数并将未用映射槽清零（PREOP，控制字保持零）：
sudo python3 scripts/probe_pdo_counts.py enp0s31f6 > build/pdo-counts.json
# 默认只做失能下的位置 PDO -> SDO 回读：
sudo python3 scripts/probe_motion.py enp0s31f6 > build/pdo-echo.json
# 显式选择真实 90 counts 往返，包含同样的失能回读检查：
sudo python3 scripts/probe_motion.py enp0s31f6 --move-unloaded-90-counts-operator-ready > build/motion.json
```

运动入口固定 CPU 2 / FIFO 60 / 2 ms，仅接受已诊断的固件 `1.6.5.0.2.1.8.8`、精确 PDO 前缀和全零尾部。先设置并核对 3% 转矩上限、窄软件位置范围和跟随误差参数；DC 稳定后，控制字为零时交替写入位置小偏移，用独立 SDO 核对收到的目标及反馈，再进入使能流程。异常不自动重试，每次退出持续发送失能帧一秒，要求连续 200 ms 的有效失能反馈；未确认时保留临时限值。完整物理急停和断线停车仍未验证。

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
