import math
import time
import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.core.factory import build_continuum_ik, build_tendon_mapper
from continuum_sdk.core.config_loader import load_continuum_config
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig

def main():
    continuum_cfg = load_continuum_config("config/continuum.yaml")
    pmac_config = PMACConfig(ip="192.168.0.200")
    robot = PMACRobotController(pmac_config)

    ik = build_continuum_ik(continuum_cfg)
    tendon_mapper = build_tendon_mapper(continuum_cfg)
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_config.pulses_per_rad,
        pulses_per_meter=pmac_config.pulses_per_meter,
        axis_order=pmac_config.axis_order,
        axis_signs=pmac_config.axis_signs
    )

    update_hz = continuum_cfg.control.update_hz
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000.0

    sin_freq_hz = 0.05
    amp_m = 0.02  # 修正幅值为合理的米级物理单位

    prev_axis_targets = None

    try:
        robot.safe_boot_and_home()

        base_pulses = robot.base_positions.copy()
        center_p, _ = ik.fk_tip()

        start_time = time.perf_counter()
        next_call = start_time

        while True:
            now = time.perf_counter()
            t = now - start_time

            # 修正 1：确保 t=0 时偏移量绝对为 0，防止 PMAC 报错急停
            p_goal = center_p + np.array(
                [
                    amp_m * math.sin(2.0 * math.pi * sin_freq_hz * t),           # X 轴（左右摆动）
                    0.0,                                                         # Y 轴（前进方向）锁定基准，不主动推拉
                    amp_m * (1.0 - math.cos(2.0 * math.pi * sin_freq_hz * t)),   # Z 轴（上下摆动）
                ],
                dtype=float,
            )

            # 1. IK 求解
            result = ik.solve(
                p_goal=p_goal,
                max_steps=continuum_cfg.ik.max_inner_steps,
            )

            # 2. 转换逻辑轴 (rad, m)
            axis_targets = tendon_mapper.to_axis_targets(result.u)

            # 3. 逻辑轴 → 脉冲 (底层映射)
            target_pulses = axis_mapper.logical_to_pulses(
                base_pulses=base_pulses,
                logical_targets=axis_targets,
            )

            # 4. 速度计算
            if prev_axis_targets is None:
                velocities = [0.0] * 5
            else:
                logical_vel = axis_mapper.diff_velocity(
                    prev_targets=prev_axis_targets,
                    next_targets=axis_targets,
                    dt_s=update_interval,
                )
                velocities = axis_mapper.logical_velocity_to_pulses_per_ms(logical_vel)

            prev_axis_targets = axis_targets

            # 5. 下发 PVT 报文给 Modbus
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=move_time_ms,
            )

            # 修正 2：加回状态监控！每半秒打印一次状态，防止盲人摸象
            if int(t * update_hz) % (update_hz // 2) == 0:
                print(
                    f"⏱️ t={t:.2f}s | "
                    f"🎯 目标 X:{p_goal[0]:.4f}m, Y:{p_goal[1]:.4f}m | "
                    f"⚙️ 脉冲增量: {target_pulses[0] - base_pulses[0]} | "
                    f"📉 误差: {np.linalg.norm(result.error):.5f}"
                )

            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()

if __name__ == "__main__":
    main()
