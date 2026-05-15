# -*- coding: utf-8 -*-
import math
import time
import numpy as np
import sys
from pathlib import Path

# 确保能找到 src 目录
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from omega_sdk.haptic_device import OmegaDevice, HapticState
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
    ik.lmbda, ik.alpha = cfg.ik.lambda_damping, cfg.ik.alpha
    ik.pos_tol, ik.dir_tol, ik.ori_tol = cfg.ik.pos_tol_m, cfg.ik.dir_tol, cfg.ik.ori_tol_rad
    return ik

def main():
    # 1. 加载配置与初始化硬件
    continuum_cfg = load_continuum_config("config/continuum.yaml")
    pmac_config = PMACConfig(ip="192.168.0.200")
    robot = PMACRobotController(pmac_config)
    omega = OmegaDevice()

    # 2. 实例化算法层
    ik = build_ik(continuum_cfg)
    tendon_mapper = ContinuumTendonMapper()
    axis_mapper = ContinuumAxisMapper(
        pulses_per_rad=pmac_config.pulses_per_rad,
        pulses_per_meter=pmac_config.pulses_per_meter,
        axis_order=pmac_config.axis_order,
        axis_signs=pmac_config.axis_signs
    )

    # 3. 遥操作映射参数
    # Omega 工作空间约为 +-0.05m，机器人弯曲空间约为 +-0.02m，推进空间 0~0.25m
    SCALE_BENDING = 0.4  # 弯曲映射比例
    SCALE_EXTENSION = 2.0 # 推进映射比例 (主手拉 10cm 对应机器人进给 20cm)
    
    update_hz = 50 # 建议保持 50Hz 提高手感
    update_interval = 1.0 / update_hz
    move_time_ms = update_interval * 1000.0

    try:
        # 连接主手
        if not omega.connect():
            print("❌ 无法连接 Omega 主手")
            return

        # 执行安全上电与回零序列 (包含防暴走清洗)
        robot.safe_boot_and_home()
        
        base_pulses = robot.base_positions.copy()
        # 获取机器人当前的初始笛卡尔位姿
        center_p, _ = ik.fk_tip()
        
        # 记录主手开启瞬间的位置，实现增量控制（防止开机跳变）
        init_haptic = omega.get_state()
        master_offset = np.array(init_haptic.pos) 

        print("\n🎮 Omega 遥操作已就绪！")
        print(f"模式: {ik.task_mode} | 频率: {update_hz}Hz")
        print("提示: 主手左右->X, 上下->Z, 前后->Y推进 | 按 Ctrl+C 退出")

        prev_axis_targets = None
        next_call = time.perf_counter()

        while True:
            # 维持稳定频率
            now = time.perf_counter()
            t_total = now - next_call
            
            # [Input] 获取主手状态
            haptic_data = omega.get_state()
            curr_master_pos = np.array(haptic_data.pos)
            
            # [Mapping] 计算主手相对位移并映射到机器人目标点
            # 映射关系: Master X -> Slave X, Master Y -> Slave Z, Master Z -> Slave Y
            rel_pos = curr_master_pos - master_offset
            
            p_goal = center_p + np.array([
                rel_pos[0] * SCALE_BENDING,  # Slave X
                rel_pos[2] * SCALE_EXTENSION, # Slave Y (推进)
                rel_pos[1] * SCALE_BENDING   # Slave Z (弯曲上下)
            ], dtype=float)

            # [Algorithm] 1. IK 求解
            result = ik.solve(p_goal=p_goal, max_steps=continuum_cfg.ik.max_inner_steps)
            
            # [Algorithm] 2. 转换为逻辑轴 (rad, m)
            axis_targets = tendon_mapper.to_axis_targets(result.u)

            # [Algorithm] 3. 逻辑轴 -> 脉冲 (底层映射)
            target_pulses = axis_mapper.logical_to_pulses(
                base_pulses=base_pulses,
                logical_targets=axis_targets,
            )

            # [Algorithm] 4. 速度计算 (PVT 核心)
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

            # [Output] 5. 下发 PVT 报文
            robot.move_pvt_stream(
                target_pulses=target_pulses,
                velocities=velocities,
                move_time=move_time_ms,
            )

            # [Monitor] 状态打印 (每 0.5s 一次)
            if int(now * update_hz) % (update_hz // 2) == 0:
                err = np.linalg.norm(result.error)
                color = "\033[92m" if err < 0.005 else "\033[91m"
                print(f"🎯 目标 X:{p_goal[0]:.3f} Y:{p_goal[1]:.3f} Z:{p_goal[2]:.3f} | 误差: {color}{err:.5f}\033[0m")

            # 严格时序控制
            next_call += update_interval
            sleep_time = next_call - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_call = time.perf_counter()

    except KeyboardInterrupt:
        print("\n⏹️ 遥操作已停止")
    except Exception as e:
        print(f"\n❌ 运行时异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        omega.close()
        robot.close()

if __name__ == "__main__":
    main()