# 2026-09-09 未接线主站加载验证

在 `feat/igh-offline` 分支继续进行真实 Linux 内核模块测试。用户确认驱动器尚未接线，测试前后有线口 `enp0s31f6` 均为 carrier=0。

| 检查 | 结果 |
| --- | --- |
| 当前内核 | `7.0.0-31-generic`，与已编译模块一致 |
| Secure Boot | disabled |
| 指定网卡 | `enp0s31f6`，MAC `38:a7:46:51:7e:48` |
| 仅加载 `ec_master` | `/dev/EtherCAT0` 建立，CLI 成功读取，Phase 为 Waiting for device(s) |
| 再加载 `ec_generic` | 指定 MAC 显示 attached，Phase 为 Idle，Active 为 no |
| 总线状态 | Link DOWN，Slaves 0，Tx/Rx 帧数均为 0 |
| 结束清理 | `ec_generic`、`ec_master` 均已卸载 |

此次验证补足了“模块可以编译，但能否加载尚未知”这一项。没有接触真实驱动器，也没有发送应用周期目标、SDO 写入、状态切换或使能命令。实际从站识别、PDO/SDO、DC 同步、实时运动后端、回零和停机仍需后续验证或实现。

复现命令（进入 `ethercat/`）：

```bash
sudo bash scripts/probe_unconnected.sh enp0s31f6
```

脚本仅用于未接线测试：拒绝已有链路、无线网卡和已加载主站模块；检查模块内核版本，临时加载并验证指定网卡已附着，退出时卸载本次加载的模块。它不会安装系统服务、设置开机启动或修改网络配置。已经接线时应使用后续的现场诊断流程。

本机原始输出：`build/unconnected-probe-2026-09-09.log`。

## 后续接线

电脑的专用有线口接驱动器链路第一台的 **EtherCAT IN**；第一台 OUT 接第二台 IN，以此类推，保持原五轴顺序。原 PMAC 主站端从这条链路断开，电脑上网继续使用 Wi-Fi。

使用适合现场的屏蔽 Cat5e/Cat6 网线直连；端口以驱动器外壳标注和实际型号手册为准。驱动器的电源与电机使能是独立事项，首次通信检查保持电机禁用。网线连接成功后，先核对从站身份与状态，再进行运动相关联调。
