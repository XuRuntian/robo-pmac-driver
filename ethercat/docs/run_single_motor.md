# 自己运行单台电机与修改角度

适用当前接法：一台空载 Diamond 连接 `enp0s31f6`，人员在场并可立即断电；不要同时运行 PMAC 或另一个主站。当前分支为 `feat/igh-offline`。

## 构建与首次准备

```bash
cd /home/xrt/水射流/robo-pmac-driver/ethercat
bash scripts/build_commissioning.sh
```

首次使用、驱动器重新上电或被 PMAC 修改映射后，准备本测试需要的 PDO，整个步骤保持失能：

```bash
sudo python3 scripts/probe_pdo_counts.py enp0s31f6 > build/pdo-setup.json
```

确认命令成功退出且 `build/pdo-setup.json` 的 `errors` 为空。不要用分号或忽略错误的方式串接准备和运动命令。

## 设置相对角度并往返

```bash
sudo python3 scripts/probe_motion.py enp0s31f6 \
  --move --angle-deg 1 --move-seconds 2 > build/motion.json
```

这会先完成驱动器检查、PDO 回读及 DC 同步，然后对齐当前编码器位置，保持一秒，以平滑轨迹用两秒移动约 +1°，保持一秒，再用两秒返回起点，保持一秒，最后失能。角度指电机轴相对起点的角度；正负表示编码器定义的两个方向。开始时需要等待检测和同步，通常不会立即转动。

| 参数 | 含义 | 默认 / 范围 |
|---|---|---|
| `--move` | 执行空载真机往返 | 不加时仅做失能回读，不使能 |
| `--angle-deg` | 相对角度；负数反向 | 默认 0.25°；非零、绝对值 ≤10°，至少可换算为 1 count |
| `--move-seconds` | 去程和回程各自的时间 | 默认 2 秒；1–5 秒 |
| `--plan` | 仅预览参数，不连接主站 | 不需要 sudo，不会运动 |

例如反方向 1°、每程三秒：

```bash
sudo python3 scripts/probe_motion.py enp0s31f6 \
  --move --angle-deg -1 --move-seconds 3 > build/motion-negative.json
```

只预览 5°、每程四秒的换算和限位：

```bash
python3 scripts/probe_motion.py enp0s31f6 \
  --move --angle-deg 5 --move-seconds 4 --plan
```

重复执行即可再做一次往返，每次从新鲜实际位置对齐起点。没有无限循环、运动后持续保持使能或只去不回的入口。输出文件重名时会覆盖上一次记录，可自行更换文件名。

`Ctrl+C` 请求停止并执行失能收尾；等待程序退出。它是软件停止请求，不替代现场断电。若停止反馈未确认，程序保留较保守的临时限值并在 JSON 中记录 `restore_skipped`，不会自动再次使能。

## 确认结果

终端显示当前请求和结束摘要，详细结果保存在重定向的 JSON 中：

```bash
python3 - <<'PY'
import json
r = json.load(open('build/motion.json'))
c = r.get('commissioning', {})
o = c.get('observation') or {}
print('错误:', r['errors'])
print('角度与时间:', c.get('plan'))
print('完成:', o.get('completed'), '确认失能:', o.get('disabled_confirmed'))
print('不完整 WKC:', o.get('bad_wkc'), '掉 OP:', o.get('bad_op'))
print('恢复原限值:', c.get('restored'), '模块卸载:', r.get('modules_unloaded'))
print('结束状态:', c.get('after'))
PY
```

正常结果为 `errors=[]`、`completed=1`、`disabled_confirmed=1`、`bad_wkc=0`、`bad_op=0`、`restored=true`、`modules_unloaded=true`。失能状态字可能是 `0x1640`，不必整字等于 `0x0240`。

如果报 PDO 映射/未用槽不匹配，先重新运行失能 PDO 准备并查看其结果。如果报编码器/传动比例变化，不要修改换算常量来跳过检查，应先核实实际参数。故障、WKC 或同步超限时先看记录，测试入口不会自动清故障或循环重试。

## 修改代码时的位置

日常改角度和时间直接用参数，无需重新编译。进一步改测试行为可查看：

- `scripts/probe_motion.py`：角度换算、命令行参数、临时驱动器限值及恢复。
- `include/continuum/commissioning_motion.hpp`：使能状态流程、平滑往返、保持时间、到位与异常判断。
- `src/motion_probe.cpp`：2 ms 总线循环、PDO/SDO 核对、发送运动命令及失能确认。

修改 C++ 后运行 `bash scripts/test_offline.sh` 和 `bash scripts/build_commissioning.sh`。位置以 counts 为单位；当前已读取并在运行前复核 131072 counts/转以及单位比例，`counts = round(angle_deg × 131072 / 360)`。例如 1° 取整为 364 counts，命令角度约 0.999756°；`command_angle_deg` 是取整后的命令值，不是测量值。

主机行程范围为 `max(256, abs(counts)+128)`，驱动器软件范围再各增加 256 counts。跟随误差和转矩限制不随角度扩大：驱动器转矩上限仍为额定值的 3%，主机跟随误差上限仍为 64 counts。不要只把某一处硬编码上限放大而遗漏其余检查。

新参数接口已经通过离线验证；截至本次修改，实机成功记录仍是旧版本的 90 counts（约 0.247°）往返。±10° 是接口允许范围，尚未逐角度完成实机验收。
