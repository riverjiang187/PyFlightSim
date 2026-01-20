"""
Main entry point for the FlightSim simulation.
Orchestrates the entire simulation pipeline: loading configs, initializing physics,
running the time loop, and logging data.

FlightSim 的主仿真入口。
负责编排整个仿真流程：加载配置、初始化物理引擎、运行时间循环、记录数据。
"""

import numpy as np
from tools.config_loader import ConfigLoader
from tools.logger import DataLogger
from modules.utils.math3d import MathUtils
from modules.dynamics.state import AircraftState
from modules.dynamics.kinematics import Kinematics, MassProperties
from modules.dynamics.integrator import Integrator
from modules.dynamics.forces import Aerodynamics, AeroParams, ControlInputs
from modules.environment.atmosphere import Atmosphere
from modules.environment.turbulence import DrydenTurbulence
from modules.sensors.imu import IMU
from modules.sensors.air_data import AirDataComputer
from modules.sensors.gps import GPS
from modules.control.autopilot import Autopilot
from modules.control.mixer import Mixer


def main():
    print("=== FlightSim: Initializing ===")

    # 1. Load Configuration / 加载配置
    try:
        ac_cfg = ConfigLoader.load_yaml("configs/aircraft.yaml")
        ap_cfg = ConfigLoader.load_yaml("configs/autopilot.yaml")
        sim_cfg = ConfigLoader.load_yaml("configs/simulation.yaml")
    except FileNotFoundError as e:
        print(f"Error loading config: {e}")
        return

    # 2. Initialize Physics Engine / 初始化物理引擎
    # Mass Properties / 质量属性
    mp = ac_cfg['mass_props']
    mass_props = MassProperties(mp['mass'], mp['Ixx'], mp['Iyy'], mp['Izz'])
    kinematics = Kinematics(mass_props)

    # Aerodynamics Model / 气动模型
    ap = ac_cfg['aero_params']
    aero_params = AeroParams(
        S=ap['S'], b=ap['b'], c=ap['c'],
        C_L_0=ap['C_L_0'], C_L_alpha=ap['C_L_alpha'],
        C_D_0=ap['C_D_0'], K=ap['K'],
        C_m_0=ap['C_m_0'], C_m_alpha=ap['C_m_alpha'], C_m_q=ap['C_m_q'],
        C_l_delta_a=ap.get('C_l_delta_a', 0.1),
        C_n_delta_r=ap.get('C_n_delta_r', 0.05),
        C_n_beta=ap.get('C_n_beta', 0.1),
        C_l_p=ap.get('C_l_p', -0.4),
        C_n_r=ap.get('C_n_r', -0.2)
    )
    aero = Aerodynamics(aero_params)

    # 3. Initialize Systems / 初始化系统
    imu = IMU()
    adc = AirDataComputer()
    # Set GPS home location (e.g., SFO Airport) / 设置 GPS 初始位置
    gps = GPS(home_lat=37.6188, home_lon=-122.3750)

    logger = DataLogger()
    autopilot = Autopilot(ap_cfg)
    mixer = Mixer(ac_cfg.get('mixer_type', 'standard'))

    # Environment Configuration / 环境配置
    env_cfg = sim_cfg.get('environment', {})
    turb_cfg = env_cfg.get('turbulence', {})
    if turb_cfg.get('enable', False):
        turb = DrydenTurbulence(intensity=turb_cfg.get('intensity'))
        print("Environment: Turbulence ON")
    else:
        turb = DrydenTurbulence(intensity=[0, 0, 0])
        print("Environment: Turbulence OFF")

    # 4. Simulation Setup / 仿真设置
    dt = sim_cfg['time']['dt']
    t_max = sim_cfg['time']['duration']
    num_steps = int(t_max / dt)

    # Set Mission Targets / 设置任务目标
    tgt_alt = sim_cfg['mission']['target_alt']
    tgt_spd = sim_cfg['mission']['target_speed']
    autopilot.set_targets(alt=tgt_alt, speed=tgt_spd, heading_deg=0.0)

    # Initialize State Vector / 初始化状态向量
    init_data = sim_cfg['initial_state']
    init_pos = np.array(init_data['pos'])
    init_vel = np.array(init_data['vel'])

    # Convert Euler to Quaternion / 欧拉角转四元数
    att_deg = init_data['att_deg']
    att_rad = np.radians(att_deg)
    init_quat = MathUtils.euler_to_quat(att_rad[0], att_rad[1], att_rad[2])

    aircraft = AircraftState(pos=init_pos, vel=init_vel, att=init_quat)
    controls = ControlInputs()

    print(f"Simulating {t_max}s | Target: {tgt_alt}m @ {tgt_spd}m/s")
    print("-" * 60)

    # 5. Main Loop / 主循环
    for step in range(num_steps):
        current_time = step * dt

        # --- Environment / 环境 ---
        # Calculate wind and atmosphere / 计算风场和大气
        wind_gusts = turb.update(-aircraft.pos[2], adc.get_reading().airspeed_tas, dt)
        rho, _, _, _ = Atmosphere.get_properties(-aircraft.pos[2])

        # --- Sensors / 感知 ---
        # Physics engine calculates true forces / 物理引擎计算真实受力
        forces_body, moments_body = aero.get_forces_and_moments(aircraft, rho, controls, wind_gusts)

        # Update sensor readings / 更新传感器读数
        imu.update(aircraft, forces_body, mass_props.mass)
        adc.update(aircraft)
        gps.update(aircraft)

        # --- GNC (Guidance, Navigation, Control) / 制导导航与控制 ---
        # Autopilot calculates logical commands / 自动驾驶仪计算逻辑指令
        p_cmd, r_cmd, y_cmd, t_cmd = autopilot.update(imu.get_data(), adc.get_reading(), dt)

        # Mixer maps to physical actuators / 混控器映射到物理舵面
        controls = mixer.mix(p_cmd, r_cmd, y_cmd, t_cmd)

        # --- Physics Integration / 物理积分 ---
        # Define differential equation wrapper / 定义微分方程包装器
        def physics_wrapper(y_vec):
            temp = AircraftState();
            temp.from_vector(y_vec)
            r, _, _, _ = Atmosphere.get_properties(-temp.pos[2])
            f, m = aero.get_forces_and_moments(temp, r, controls, wind_gusts)
            return kinematics.get_state_derivative(temp, f, m)

        # Execute RK4 step / 执行 RK4 积分
        next_vec = Integrator.rk4_step(physics_wrapper, aircraft.to_vector(), dt)
        aircraft.from_vector(next_vec)

        # --- Logging / 记录 ---
        logger.log(current_time, aircraft, controls, adc.get_reading(),
                   imu.get_data().specific_force[2], gps.get_reading())

        # Console Progress / 控制台输出
        if step % int(5.0 / dt) == 0:
            g_read = gps.get_reading()
            print(f"T={current_time:5.1f} | Alt={g_read.altitude:7.1f} | Lat={g_read.latitude:.4f}")

    # 6. Finalize / 结束
    logger.save_to_csv("flight_data.csv")
    print("=== Simulation Complete ===")


if __name__ == "__main__":
    main()