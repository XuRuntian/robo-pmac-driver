import math
import time
import numpy as np

from continuum_sdk.control.axis_mapper import ContinuumAxisMapper
from continuum_sdk.control.tendon_mapper import ContinuumTendonMapper
from continuum_sdk.core.config_loader import load_continuum_config
from continuum_sdk.kinematics.dls_ik import DLSIK
from continuum_sdk.kinematics.geometry import ContinuumGeometry
from pmac_sdk.controller.robot_api import PMACRobotController
from pmac_sdk.core.config_model import PMACConfig

def build_ik(cfg) -> DLSIK:
    geometry = ContinuumGeometry(
        s_s=cfg.geometry.s_s,
        s_a=cfg.geometry.s_a,
        s_c=cfg.geometry.s_c,
        h_bc=cfg.geometry.h_bc,
        h_de=cfg.geometry.h_de,
        theta_a_max=cfg.geometry.theta_a_max_rad,
        theta_c_max=cfg.geometry.theta_c_max_rad,
        d_min=cfg.geometry.d_min_m,
        d_max=cfg.geometry.d_max_m,
        base_offset=cfg.geometry.base_offset_m,
    )

    ik = DLSIK(geometry=geometry, task_mode=cfg.ik.task_mode)
    ik.lmbda = cfg.ik.lambda_damping
    ik.alpha = cfg.ik.alpha
    ik.pos_tol = cfg.ik.pos_tol_m
    ik.dir_tol = cfg.ik.dir_tol
    ik.ori_tol = cfg.ik.ori_tol_rad

    return ik

def main():
    continuum_cfg = load_continuum_config("config/continuum.yaml")
    pmac_config = PMACConfig(ip="192.168.0.200")
    robot = PMACRobotController(pmac_config)

    ik = build_ik(continuum_cfg)
    tendon_mapper = ContinuumTendonMapper()
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_config.pulses_per_rad,
        pulses_per_meter=pmac_config.pulses_per_meter,
        axis_order=pmac_config.axis_order,
        axis_signs=pmac_config.axis_signs
    )

    update_hz = continuum_cfg.control.update_hz
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000.0

    sin_freq_hz = 0.1
    amp_m = 1  # 修正幅值为合理的米级物理单位

    prev_axis_targets = None

    try:
        robot.axi_syn_boot()
        time.sleep(2.0)
        robot.connect_and_home()

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
                    amp_m * math.sin(2.0 * math.pi * sin_freq_hz * t),
                    amp_m * (1.0 - math.cos(2.0 * math.pi * sin_freq_hz * t)), # 1 - cos 起点为 0
                    0.0,
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