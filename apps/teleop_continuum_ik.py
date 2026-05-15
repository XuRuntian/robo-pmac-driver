# apps/teleop_continuum_omega.py
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
from omega_sdk.haptic_device import OmegaDevice, HapticState

# ----------------------
# IK 构造函数
# ----------------------
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

# ----------------------
# Omega 主手映射算法
# ----------------------
class OmegaToCartesianIK:
    """
    将 Omega HapticState 转换为连续体末端目标 p_goal (XYZ m) + 夹爪
    """
    def __init__(self, center_p: np.ndarray, scale_xyz=(0.05, 0.05, 0.05)):
        self.center_p = center_p
        self.scale = np.array(scale_xyz, dtype=float)

    def solve(self, haptic_state: HapticState) -> tuple[np.ndarray, float]:
        """
        输入: Omega HapticState
        输出: p_goal (XYZ m), gripper (0~1)
        """
        # 假设主手的 x/y/z [-1,1] 映射到 末端 XYZ 偏移
        delta = np.array([
            haptic_state.pos[0] * self.scale[0],
            haptic_state.pos[1] * self.scale[1],
            haptic_state.pos[2] * self.scale[2],
        ], dtype=float)
        p_goal = self.center_p + delta
        gripper = haptic_state.gripper_deg / 100.0  # 归一化到 0~1
        return p_goal, gripper

# ----------------------
# 主函数
# ----------------------
def main():
    continuum_cfg = load_continuum_config("config/continuum.yaml")
    pmac_config = PMACConfig(ip="192.168.0.200")
    robot = PMACRobotController(pmac_config)

    ik = build_ik(continuum_cfg)
    tendon_mapper = ContinuumTendonMapper()
    axis_mapper = AxisMapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_config.pulses_per_rad,
        pulses_per_meter=pmac_config.pulses_per_meter,
        axis_order=pmac_config.axis_order,
        axis_signs=pmac_config.axis_signs,
    )

    update_hz = continuum_cfg.control.update_hz
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000.0

    prev_axis_targets = None

    # ----------------------
    # Omega 初始化
    # ----------------------
    omega = OmegaDevice()
    if not omega.connect():
        print("❌ Omega 连接失败")
        return

    try:
        # ----------------------
        # PMAC 上电/复位/归零
        # ----------------------
        robot.safe_boot_and_home()
        base_pulses = robot.base_positions.copy()
        center_p, _ = ik.fk_tip()
        omega_mapper = OmegaToCartesianIK(center_p=center_p)

        print("\n🚀 启动 Omega 主手遥操作 (Ctrl+C 停止)")

        start_time = time.perf_counter()
        next_call = start_time

        while True:
            t = time.perf_counter() - start_time
            # 1. 获取主手状态
            haptic_state = omega.get_state()

            # 2. Omega 映射到末端目标
            p_goal, gripper = omega_mapper.solve(haptic_state)

            # 3. IK 求解
            result = ik.solve(p_goal=p_goal, max_steps=continuum_cfg.ik.max_inner_steps)

            # 4. tendon 映射
            axis_targets = tendon_mapper.to_axis_targets(result.u)
            # TODO: 可以考虑把 gripper 映射到第5轴 d 或夹爪逻辑

            # 5. 逻辑轴 → 脉冲
            target_pulses = axis_mapper.logical_to_pulses(
                base_pulses=base_pulses,
                logical_targets=axis_targets,
            )

            # 6. 速度
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

            # 7. 下发 PVT
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=move_time_ms,
            )

            # 8. 打印状态
            if int(t * update_hz) % (update_hz // 2) == 0:
                print(
                    f"⏱ t={t:.2f}s | "
                    f"p_goal={p_goal.round(4)} | "
                    f"u={np.round(result.u,3)} | "
                    f"err={np.linalg.norm(result.error):.5f}"
                )

            # 控制周期
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()

    except KeyboardInterrupt:
        print("\n⏹️ 已停止遥操作")
    finally:
        omega.close()
        robot.close()

if __name__ == "__main__":
    main()