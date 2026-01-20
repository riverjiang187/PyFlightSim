# Project Export: FlightSim

## Project Structure

```text
FlightSim/
├── main.py
├── plot_results.py
│   tools/
│   │   ├── __init__.py
│   │   ├── config_loader.py
│   │   ├── logger.py
│   configs/
│   │   ├── aircraft.yaml
│   │   ├── autopilot.yaml
│   │   ├── simulation.yaml
│   modules/
│   │   ├── __init__.py
│   │   sensors/
│   │   │   ├── __init__.py
│   │   │   ├── air_data.py
│   │   │   ├── gps.py
│   │   │   ├── imu.py
│   │   dynamics/
│   │   │   ├── __init__.py
│   │   │   ├── forces.py
│   │   │   ├── integrator.py
│   │   │   ├── kinematics.py
│   │   │   ├── state.py
│   │   utils/
│   │   │   ├── __init__.py
│   │   │   ├── constants.py
│   │   │   ├── math3d.py
│   │   environment/
│   │   │   ├── __init__.py
│   │   │   ├── atmosphere.py
│   │   │   ├── turbulence.py
│   │   control/
│   │   │   ├── __init__.py
│   │   │   ├── autopilot.py
│   │   │   ├── mixer.py
│   │   │   ├── pid.py

```

---

## File Contents

### File: `main.py`

```python
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
```

---

### File: `plot_results.py`

```python
"""
Data visualization tool.
Reads 'flight_data.csv' and generates a dashboard with 12 subplots.
Covers trajectory, attitude, speed, aerodynamics, and controls.

数据可视化工具。
读取 flight_data.csv 并生成包含 12 个子图的飞行分析仪表板。
涵盖：轨迹、姿态、速度、气动角、控制量等。
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot_flight_data(filename="flight_data.csv"):
    print(f"Reading {filename}...")
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print("File not found."); return

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'ggplot')
    fig, axs = plt.subplots(4, 3, figsize=(18, 12))
    fig.suptitle(f'Full Flight Analysis: {filename}', fontsize=16)

    # Row 1: Navigation & Performance / 导航与性能
    axs[0, 0].plot(df['Time'], df['Alt'], 'b');
    axs[0, 0].set_title('Altitude (m)')

    if 'Lat' in df.columns:
        axs[0, 1].plot(df['Lon'], df['Lat'], 'purple')
        axs[0, 1].plot(df['Lon'].iloc[0], df['Lat'].iloc[0], 'go', label='Start')
        axs[0, 1].plot(df['Lon'].iloc[-1], df['Lat'].iloc[-1], 'rx', label='End')
        axs[0, 1].set_title('GPS Ground Track');
        axs[0, 1].axis('equal')
        axs[0, 1].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].legend()
    else:
        axs[0, 1].plot(df['PosY'], df['PosX'], 'purple');
        axs[0, 1].set_title('Local Track')

    axs[0, 2].plot(df['Time'], df['TAS'], 'g');
    axs[0, 2].set_title('TAS (m/s)')

    # Row 2: Attitude / 姿态
    axs[1, 0].plot(df['Time'], df['Roll'], 'tab:orange');
    axs[1, 0].set_title('Roll (deg)')
    axs[1, 1].plot(df['Time'], df['Pitch'], 'r');
    axs[1, 1].set_title('Pitch (deg)')
    axs[1, 2].plot(df['Time'], df['Yaw'], 'tab:brown');
    axs[1, 2].set_title('Yaw (deg)')

    # Row 3: Aerodynamics & G-Load / 气动与过载
    axs[2, 0].plot(df['Time'], df['Alpha'], 'm');
    axs[2, 0].set_title('Alpha (deg)')
    if 'Beta' in df.columns: axs[2, 1].plot(df['Time'], df['Beta'], 'c'); axs[2, 1].set_title('Beta (deg)')
    axs[2, 2].plot(df['Time'], df['AccZ'], 'k');
    axs[2, 2].set_title('Acc Z (m/s^2)')

    # Row 4: Controls / 控制量
    axs[3, 0].plot(df['Time'], df['Elevator'], 'k');
    axs[3, 0].set_title('Elevator')
    if 'Aileron' in df.columns:
        axs[3, 1].plot(df['Time'], df['Aileron'], 'tab:blue', label='Ail')
        axs[3, 1].plot(df['Time'], df['Rudder'], 'tab:orange', label='Rud', linestyle='--')
        axs[3, 1].legend();
        axs[3, 1].set_title('Aileron & Rudder')
    axs[3, 2].plot(df['Time'], df['Throttle'], 'orange');
    axs[3, 2].set_title('Throttle')

    plt.tight_layout();
    plt.show()


if __name__ == "__main__":
    plot_flight_data()
```

---

### File: `tools/__init__.py`

```python
"""
Application-level utilities.
Contains tools for configuration loading and data logging.

应用程序级别的辅助工具。
包含配置加载和数据记录工具。
"""
```

---

### File: `tools/config_loader.py`

```python
"""
Utility to load configuration parameters from YAML files.

负责从 YAML 文件加载配置参数。
"""
import yaml
import os

class ConfigLoader:
    @staticmethod
    def load_yaml(filepath: str):
        """
        Loads a YAML file and returns a dictionary.
        加载 YAML 文件并返回字典。
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Config file not found: {filepath}")
        with open(filepath, 'r') as f:
            return yaml.safe_load(f)
```

---

### File: `tools/logger.py`

```python
"""
Flight Data Recorder (FDR).
Captures simulation state, controls, and sensor data to CSV format.

飞行数据记录器 (FDR)。
负责将仿真过程中的状态、控制量、传感器数据保存为 CSV 格式。
"""
import csv
import numpy as np


class DataLogger:
    def __init__(self):
        self.data = []
        self.headers = [
            "Time", "PosX", "PosY", "Alt",
            "VelN", "VelE", "VelD",
            "Roll", "Pitch", "Yaw",
            "TAS", "Alpha", "Beta",
            "Elevator", "Aileron", "Rudder", "Throttle",
            "AccZ", "Lat", "Lon", "GPS_Alt", "GPS_Spd"
        ]

    def log(self, time, state, controls, air_data, acc_z, gps_data):
        """
        Captures a single frame of simulation data.
        记录单帧仿真数据。
        """
        euler_deg = np.degrees(state.euler_angles)
        alpha_deg = np.degrees(air_data.alpha)
        beta_deg = np.degrees(getattr(air_data, 'beta', 0.0))

        row = [
            f"{time:.4f}",
            f"{state.pos[0]:.4f}", f"{state.pos[1]:.4f}", f"{-state.pos[2]:.4f}",
            f"{state.vel[0]:.4f}", f"{state.vel[1]:.4f}", f"{state.vel[2]:.4f}",
            f"{euler_deg[0]:.4f}", f"{euler_deg[1]:.4f}", f"{euler_deg[2]:.4f}",
            f"{air_data.airspeed_tas:.4f}", f"{alpha_deg:.4f}", f"{beta_deg:.4f}",
            f"{controls.elevator:.4f}", f"{controls.aileron:.4f}", f"{controls.rudder:.4f}", f"{controls.throttle:.4f}",
            f"{acc_z:.4f}",
            f"{gps_data.latitude:.6f}", f"{gps_data.longitude:.6f}",
            f"{gps_data.altitude:.2f}", f"{gps_data.ground_speed:.2f}"
        ]
        self.data.append(row)

    def save_to_csv(self, filename="flight_data.csv"):
        """
        Writes buffer to disk.
        将缓冲区数据写入磁盘。
        """
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.headers)
                writer.writerows(self.data)
            print(f"[Logger] Data saved to {filename}")
        except Exception as e:
            print(f"[Logger] Error saving CSV: {e}")
```

---

### File: `configs/aircraft.yaml`

```yaml
# Aircraft Physical Properties
# 飞机物理属性

name: "Cessna 172 Skyhawk"
mixer_type: "standard" # standard, delta, v-tail

mass_props:
  mass: 1300.0
  Ixx: 1000.0
  Iyy: 4000.0
  Izz: 4500.0

aero_params:
  S: 16.2         # Wing Area / 翼面积
  b: 10.9         # Wingspan / 翼展
  c: 1.5          # Chord / 弦长

  # Longitudinal / 纵向
  C_L_0: 0.25
  C_L_alpha: 4.5
  C_D_0: 0.025
  K: 0.05
  C_m_0: 0.02
  C_m_alpha: -0.5
  C_m_q: -10.0

  # Lateral / 横侧向
  C_l_delta_a: 0.1
  C_n_delta_r: 0.05
  C_n_beta: 0.1
  C_l_p: -0.4
  C_n_r: -0.2
```

---

### File: `configs/autopilot.yaml`

```yaml
# Autopilot Configuration parameters.
# Tuned for "Limousine Mode" - Maximum smoothness and stability.
# Sacrifices response speed for passenger comfort and mechanical safety.
#
# 自动驾驶仪配置参数。
# 针对“豪华轿车模式”调优 - 追求最大的平滑度和稳定性。
# 牺牲响应速度以换取乘客舒适度和机械安全性。

# Design cruise speed for gain scheduling (m/s)
# 用于增益调度的设计巡航速度 (米/秒)
design_speed: 60.0

# --- Flight Envelope Limits / 飞行包线限制 ---
limits:
  max_climb_rate: 3.0   # m/s (Reduced for comfort / 降低以提高舒适度)
  max_sink_rate: 3.0    # m/s
  max_bank: 0.35        # rad (~20 deg, gentle turns / 柔和转弯)
  max_pitch: 0.20       # rad (~11 deg, prevent steep climb / 防止大角度爬升)

# --- Longitudinal Loops / 纵向回路 ---

# 1. Outer Loop: Altitude -> Target Climb Rate
#    外环：高度 -> 目标爬升率
altitude_hold:
  kp: 0.05    # Very low gain for gentle capture / 极低增益，实现柔和捕获
  ki: 0.0005  # Slow integration to remove error / 缓慢积分以消除误差
  kd: 0.0
  out_min: -3.0
  out_max: 3.0

# 2. Middle Loop: Climb Rate -> Target Pitch
#    中环：爬升率 -> 目标俯仰角
climb_rate_hold:
  kp: 0.02    # Low gain to prevent overshooting / 低增益以防止超调
  ki: 0.005
  kd: 0.0
  out_min: -0.20
  out_max: 0.20

# 3. Inner Loop: Pitch -> Elevator
#    内环：俯仰角 -> 升降舵
pitch_hold:
  kp: -0.5    # Reduced sensitivity / 降低灵敏度
  ki: -0.02
  kd: -0.01   # Minimal damping to stop bounce, low enough to avoid jitter / 极小阻尼防止弹跳，且不引起颤振
  out_min: -1.0
  out_max: 1.0

# --- Speed Control / 速度控制 ---
speed_hold:
  kp: 0.05    # Slow throttle response / 缓慢的油门响应
  ki: 0.005
  kd: 0.0
  out_min: 0.0
  out_max: 1.0

# --- Lateral Loops / 横侧向回路 ---

# 1. Outer Loop: Heading -> Target Roll
#    外环：航向 -> 目标滚转角
heading_hold:
  kp: 0.8     # Gentle turning / 柔和转向
  ki: 0.0
  kd: 0.0
  out_min: -0.35
  out_max: 0.35

# 2. Inner Loop: Roll -> Aileron
#    内环：滚转角 -> 副翼
roll_hold:
  kp: 0.05    # Low sensitivity / 低灵敏度
  ki: 0.005
  kd: 0.01    # Slight damping for roll stability / 轻微阻尼以保持滚转稳定
  out_min: -1.0
  out_max: 1.0

# 3. Stability Augmentation: Yaw Rate -> Rudder (Yaw Damper)
#    增稳：偏航角速度 -> 方向舵 (偏航阻尼器)
yaw_damper:
  kp: 0.5     # Moderate damping / 适度阻尼
  ki: 0.0
  kd: 0.0
  out_min: -0.3
  out_max: 0.3
```

---

### File: `configs/simulation.yaml`

```yaml
# Simulation Scenario Configuration
# 仿真场景配置

time:
  dt: 0.02            # Integration time step (s) / 积分步长
  duration: 160.0      # Total duration (s) / 总时长

initial_state:
  # NED Frame (m) / 北东下坐标系
  pos: [0.0, 0.0, -1900.0]
  # Body Frame Velocity (m/s) / 机体轴速度
  vel: [60.0, 0.0, 0.0]
  # Euler Angles [Roll, Pitch, Yaw] (deg) / 欧拉角
  att_deg: [0.0, 0.0, 0.0]

mission:
  target_alt: 2000.0  # Target Altitude (m) / 目标高度
  target_speed: 60.0  # Target Speed (m/s) / 目标速度

environment:
  turbulence:
    enable: true
    # Intensity Sigma [u, v, w] / 湍流强度
    intensity: [0.6, 0.6, 0.5]
```

---

### File: `modules/__init__.py`

```python
"""
Core library of FlightSim.
Contains dynamics, sensors, control laws, and environment models.

FlightSim 的核心库。
包含动力学、传感器、控制律和环境模型。
"""
```

---

### File: `modules/sensors/__init__.py`

```python
"""
Sensor simulation. Converts physical truth to sensor readings.

传感器仿真。负责将物理真值转换为传感器读数。
"""
```

---

### File: `modules/sensors/air_data.py`

```python
"""
Air Data Computer (ADC).
Outputs Airspeed, Barometric Altitude, Climb Rate, Alpha, and Beta.

大气数据计算机 (ADC)。
输出空速、气压高度、爬升率、攻角和侧滑角。
"""

import numpy as np
from dataclasses import dataclass
from modules.utils.math3d import MathUtils

@dataclass
class AirDataReading:
    airspeed_tas: float = 0.0
    altitude_baro: float = 0.0
    climb_rate: float = 0.0    # [NEW] m/s
    alpha: float = 0.0
    beta: float = 0.0

class AirDataComputer:
    def __init__(self):
        self.reading = AirDataReading()

    def update(self, state):
        # 1. Airspeed / 空速
        self.reading.airspeed_tas = np.linalg.norm(state.vel)

        # 2. Altitude / 高度
        self.reading.altitude_baro = -state.pos[2]

        # 3. Climb Rate / 爬升率
        # We need Vertical Velocity in NED frame (Down is positive)
        # Climb Rate = -Vel_Down_NED
        # Transform Body Velocity to NED
        # 我们需要 NED 坐标系下的垂直速度（向下为正）
        # 爬升率 = -Vel_Down_NED
        # 将机体速度转换到 NED
        R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
        vel_ned = R_b_n @ state.vel
        self.reading.climb_rate = -vel_ned[2]

        # 4. Flow Angles / 气流角
        u, v, w = state.vel
        if self.reading.airspeed_tas > 0.1:
            self.reading.alpha = np.arctan2(w, u)
            self.reading.beta = np.arcsin(np.clip(v / self.reading.airspeed_tas, -1, 1))
        else:
            self.reading.alpha = 0.0; self.reading.beta = 0.0

    def get_reading(self): return self.reading
```

---

### File: `modules/sensors/gps.py`

```python
"""
Global Positioning System (GPS).
Converts local NED coordinates to Geodetic (Lat/Lon/Alt).

全球定位系统 (GPS)。
将局部 NED 坐标转换为大地坐标 (经纬度)。
"""
import numpy as np
from dataclasses import dataclass
from modules.utils.math3d import MathUtils


@dataclass
class GPSReading:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    ground_speed: float = 0.0
    ground_course: float = 0.0


class GPS:
    def __init__(self, home_lat=37.7749, home_lon=-122.4194):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.reading = GPSReading()
        self.R_EARTH = 6378137.0
        self.DEG_TO_RAD = np.pi / 180.0
        self.RAD_TO_DEG = 180.0 / np.pi

    def update(self, state):
        # Flat Earth Projection / 平地投影近似
        d_lat_rad = state.pos[0] / self.R_EARTH
        self.reading.latitude = self.home_lat + (d_lat_rad * self.RAD_TO_DEG)

        scale = np.cos(self.home_lat * self.DEG_TO_RAD)
        d_lon_rad = state.pos[1] / (self.R_EARTH * scale)
        self.reading.longitude = self.home_lon + (d_lon_rad * self.RAD_TO_DEG)

        self.reading.altitude = -state.pos[2]

        # Ground Speed Calculation / 地速计算
        R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
        vel_ned = R_b_n @ state.vel
        vn, ve = vel_ned[0], vel_ned[1]
        self.reading.ground_speed = np.sqrt(vn ** 2 + ve ** 2)
        self.reading.ground_course = np.degrees(np.arctan2(ve, vn)) % 360.0

    def get_reading(self): return self.reading
```

---

### File: `modules/sensors/imu.py`

```python
"""
Inertial Measurement Unit (IMU).
Outputs Specific Force (Accel), Angular Rates (Gyro), and Attitude (Quat/Euler).

惯性测量单元 (IMU)。
输出比力 (加速度计)、角速率 (陀螺仪) 和姿态 (四元数/欧拉角)。
"""

import numpy as np
from dataclasses import dataclass, field

@dataclass
class IMUData:
    specific_force: np.ndarray = field(default_factory=lambda: np.zeros(3))
    angular_rates: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quat: np.ndarray = field(default_factory=lambda: np.array([1., 0., 0., 0.])) # Added for control

class IMU:
    def __init__(self):
        self.data = IMUData()

    def update(self, state, forces_body_nongrav, mass):
        self.data.angular_rates = state.rates.copy()
        # Accel measures non-gravitational forces / 加速度计测量非重力
        self.data.specific_force = forces_body_nongrav / mass if mass > 0 else np.zeros(3)

        # Output both Euler (for logging) and Quat (for control)
        # 同时输出欧拉角（用于记录）和四元数（用于控制）
        self.data.euler = state.euler_angles
        self.data.quat = state.q.copy()

    def get_data(self): return self.data
```

---

### File: `modules/dynamics/__init__.py`

```python
"""
Physics kernel.
Contains state definitions, kinematics equations, integrators, and force models.

物理内核。
包含状态定义、运动学方程、积分器和力学模型。
"""
```

---

### File: `modules/dynamics/forces.py`

```python
"""
Aerodynamics and Thrust calculation module.
Computes Lift, Drag, and Moments based on Stability Derivatives.

气动力与推力计算模块。
基于稳定性导数计算升力、阻力和力矩。
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class AeroParams:
    S: float;
    b: float;
    c: float
    C_L_0: float;
    C_L_alpha: float
    C_D_0: float;
    K: float
    C_m_0: float;
    C_m_alpha: float;
    C_m_q: float
    C_l_delta_a: float = 0.1
    C_n_delta_r: float = 0.05
    C_n_beta: float = 0.1
    C_l_p: float = -0.4
    C_n_r: float = -0.2


@dataclass
class ControlInputs:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0


class Aerodynamics:
    def __init__(self, params: AeroParams):
        self.params = params

    def get_forces_and_moments(self, state, density, controls, wind_body_vector=np.zeros(3)):
        # Calculate Airspeed (Body Vel - Wind Vel)
        # 计算空速 (机体速度 - 风速)
        v_air_vec = state.vel - wind_body_vector
        u, v, w = v_air_vec
        V_sq = u ** 2 + v ** 2 + w ** 2
        V_tas = np.sqrt(V_sq)

        if V_tas < 0.1: return np.zeros(3), np.zeros(3)

        # Alpha & Beta / 攻角与侧滑角
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / V_tas, -1, 1))
        q_bar = 0.5 * density * V_sq

        # Longitudinal Coeffs / 纵向系数
        C_L = self.params.C_L_0 + self.params.C_L_alpha * alpha + (0.5 * controls.elevator)
        C_D = self.params.C_D_0 + self.params.K * (C_L ** 2)
        q_hat = (self.params.c * state.rates[1]) / (2 * V_tas)
        C_m = (self.params.C_m_0 + self.params.C_m_alpha * alpha +
               self.params.C_m_q * q_hat + -1.5 * controls.elevator)

        # Lateral Coeffs / 横侧向系数
        p_hat = (self.params.b * state.rates[0]) / (2 * V_tas)
        r_hat = (self.params.b * state.rates[2]) / (2 * V_tas)

        C_l = (self.params.C_l_delta_a * controls.aileron) + (self.params.C_l_p * p_hat)
        C_n = (self.params.C_n_delta_r * controls.rudder) + \
              (self.params.C_n_beta * beta) + (self.params.C_n_r * r_hat)

        # Forces (Stability -> Body Frame) / 力 (稳定性坐标系 -> 机体坐标系)
        L = q_bar * self.params.S * C_L
        D = q_bar * self.params.S * C_D
        Fx = -D * np.cos(alpha) + L * np.sin(alpha)
        Fz = -D * np.sin(alpha) - L * np.cos(alpha)

        # Thrust / 推力
        max_thrust = 5000.0
        Fx += max_thrust * controls.throttle

        # Moments / 力矩
        Roll = q_bar * self.params.S * self.params.b * C_l
        Pitch = q_bar * self.params.S * self.params.c * C_m
        Yaw = q_bar * self.params.S * self.params.b * C_n

        return np.array([Fx, 0.0, Fz]), np.array([Roll, Pitch, Yaw])
```

---

### File: `modules/dynamics/integrator.py`

```python
"""
Numerical Integrator. Currently implements Runge-Kutta 4 (RK4).

数值积分器。目前实现四阶龙格-库塔法 (RK4)。
"""
class Integrator:
    @staticmethod
    def rk4_step(deriv_func, y, dt):
        """
        Standard RK4 solver.
        标准 RK4 求解器。
        """
        k1 = deriv_func(y)
        k2 = deriv_func(y + (dt * 0.5) * k1)
        k3 = deriv_func(y + (dt * 0.5) * k2)
        k4 = deriv_func(y + dt * k3)
        return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
```

---

### File: `modules/dynamics/kinematics.py`

```python
"""
Calculates Rigid Body Equations of Motion (EOM).
Includes Translational (Newton) and Rotational (Euler) equations.

计算刚体动力学微分方程。
包含平动 (牛顿) 和转动 (欧拉) 方程。
"""
import numpy as np
from modules.utils.math3d import MathUtils
from modules.utils.constants import GRAVITY


class MassProperties:
    def __init__(self, mass, Ixx, Iyy, Izz):
        self.mass = mass
        self.inertia = np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])
        self.inertia_inv = np.linalg.inv(self.inertia)


class Kinematics:
    def __init__(self, mass_props):
        self.mass_props = mass_props

    def get_state_derivative(self, state, forces_body, moments_body):
        """
        Calculates dx/dt given current state x and inputs u (forces).
        根据当前状态和输入力计算状态导数。
        """
        vel_b = state.vel
        q = state.q
        omega_b = state.rates

        # 1. Kinematics (Pos & Att) / 运动学 (位置与姿态)
        R_b_n = MathUtils.quat_to_rotation_matrix(q)
        pos_dot = R_b_n @ vel_b
        quat_dot = MathUtils.quat_derivative(q, omega_b)

        # 2. Dynamics (Vel & Rates) / 动力学 (速度与角速度)
        # Gravity in Body Frame / 机体坐标系下的重力
        g_ned = np.array([0, 0, GRAVITY])
        g_body = R_b_n.T @ g_ned

        # F = ma -> a = F/m
        F_total_b = forces_body + (g_body * self.mass_props.mass)
        coriolis = np.cross(omega_b, vel_b)
        vel_dot = (F_total_b / self.mass_props.mass) - coriolis

        # M = I*alpha + w x I*w
        J = self.mass_props.inertia
        J_inv = self.mass_props.inertia_inv
        gyroscopic = np.cross(omega_b, (J @ omega_b))
        rates_dot = J_inv @ (moments_body - gyroscopic)

        return np.concatenate([pos_dot, vel_dot, quat_dot, rates_dot])
```

---

### File: `modules/dynamics/state.py`

```python
"""
Defines the AircraftState class, the Single Source of Truth for simulation.
Contains Position, Velocity, Attitude (Quaternion), and Angular Rates.

定义 AircraftState 类，作为仿真的单一事实来源。
包含位置、速度、姿态（四元数）和角速度。
"""
import numpy as np
from modules.utils.math3d import MathUtils


class AircraftState:
    def __init__(self, pos=None, vel=None, att=None, rates=None):
        # Pos: NED Frame (m) / 位置 (北东下)
        self._pos = pos.astype(float) if pos is not None else np.zeros(3)
        # Vel: Body Frame (m/s) / 速度 (机体轴)
        self._vel = vel.astype(float) if vel is not None else np.zeros(3)
        # Att: Quaternion [w, x, y, z] / 姿态 (四元数)
        self._q = att.astype(float) if att is not None else np.array([1., 0., 0., 0.])
        # Rates: Body Frame (rad/s) / 角速度 (机体轴)
        self._rates = rates.astype(float) if rates is not None else np.zeros(3)

        self._q = MathUtils.normalize_quat(self._q)

    @property
    def pos(self): return self._pos

    @property
    def vel(self): return self._vel

    @property
    def q(self): return self._q

    @property
    def rates(self): return self._rates

    @property
    def euler_angles(self): return MathUtils.quat_to_euler(self._q)

    def to_vector(self):
        """
        Serialize state to 13-element vector for integrator.
        将状态序列化为 13 维向量以供积分器使用。
        """
        return np.concatenate([self._pos, self._vel, self._q, self._rates])

    def from_vector(self, vec):
        """
        Update state from vector.
        从向量更新状态。
        """
        self._pos = vec[0:3]
        self._vel = vec[3:6]
        self._q = MathUtils.normalize_quat(vec[6:10])
        self._rates = vec[10:13]
```

---

### File: `modules/utils/__init__.py`

```python
"""
Core mathematical utilities and physical constants.

核心数学工具与物理常数。
"""
```

---

### File: `modules/utils/constants.py`

```python
"""
Physical constants definition (SI Units).

物理常数定义 (国际单位制)。
"""
GRAVITY = 9.80665  # m/s^2
```

---

### File: `modules/utils/math3d.py`

```python
"""
3D Math Utility Library.
Handles Quaternion <-> Euler conversions and Rotation Matrix (DCM) calculations.
Includes quaternion algebra for control error computation.

3D 数学工具库。
主要处理四元数与欧拉角的转换，以及坐标系旋转矩阵的计算。
包含用于计算控制误差的四元数代数运算。

Convention / 约定:
- Quaternions: Scalar-first [w, x, y, z] / 实部在前
- Euler: [Roll, Pitch, Yaw] (Radians) / 弧度制
"""

import numpy as np

class MathUtils:
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        """
        Converts Euler angles to Quaternion [w, x, y, z].
        将欧拉角转换为四元数。
        """
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ])

    @staticmethod
    def quat_to_euler(q):
        """
        Converts Quaternion to Euler angles [roll, pitch, yaw].
        将四元数转换为欧拉角。
        """
        q0, q1, q2, q3 = q
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q0 * q2 - q3 * q1)
        if np.abs(sinp) >= 1: pitch = np.copysign(np.pi / 2, sinp)
        else: pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([roll, pitch, yaw])

    @staticmethod
    def normalize_quat(q):
        """
        Ensures quaternion is unit length.
        确保四元数为单位长度。
        """
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

    @staticmethod
    def quat_to_rotation_matrix(q):
        """
        Calculates Rotation Matrix (Body -> NED) from Quaternion.
        计算从机体坐标系到地面坐标系的旋转矩阵。
        """
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2*(q2**2 + q3**2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3),       1 - 2*(q1**2 + q3**2),   2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),   1 - 2*(q1**2 + q2**2)]
        ])

    @staticmethod
    def quat_derivative(q, rates):
        """
        Calculates dq/dt = 0.5 * q * omega.
        计算四元数导数。
        """
        p, q_rate, r = rates
        omega_mat = np.array([
            [0, -p, -q_rate, -r],
            [p,  0,  r, -q_rate],
            [q_rate, -r,  0,  p],
            [r,  q_rate, -p,  0]
        ])
        return 0.5 * (omega_mat @ q)

    @staticmethod
    def quat_conjugate(q):
        """
        Returns the conjugate (inverse for unit quats) of q.
        返回四元数的共轭（对于单位四元数即为逆）。
        """
        return np.array([q[0], -q[1], -q[2], -q[3]])

    @staticmethod
    def quat_multiply(q1, q2):
        """
        Multiplies two quaternions.
        四元数乘法。
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def get_body_frame_error(q_current, q_target):
        """
        Calculates the rotation error vector in Body Frame.
        Returns [roll_err, pitch_err, yaw_err] in radians.

        计算机体坐标系下的旋转误差向量。
        返回 [滚转误差, 俯仰误差, 偏航误差] (弧度)。
        """
        # q_error = q_current_inv * q_target
        q_curr_inv = MathUtils.quat_conjugate(q_current)
        q_err = MathUtils.quat_multiply(q_curr_inv, q_target)

        # Normalize to be safe / 归一化以确保安全
        q_err = MathUtils.normalize_quat(q_err)

        # Handle double cover (q and -q are same rotation)
        # Ensure we take the shortest path
        # 处理双倍覆盖问题（q 和 -q 表示相同的旋转），确保走最短路径
        if q_err[0] < 0:
            q_err = -q_err

        # Extract vector part for small angle approximation or full conversion
        # 提取向量部分
        # Roll error (x), Pitch error (y), Yaw error (z)
        # Using 2*atan2 is robust for large angles
        return np.array([
            2.0 * np.arctan2(q_err[1], q_err[0]),
            2.0 * np.arctan2(q_err[2], q_err[0]),
            2.0 * np.arctan2(q_err[3], q_err[0])
        ])
```

---

### File: `modules/environment/__init__.py`

```python
"""
Environment models (Atmosphere, Wind, Gravity).

环境模型（大气、风场、重力场等）。
"""
```

---

### File: `modules/environment/atmosphere.py`

```python
"""
ISA (International Standard Atmosphere) Model.
Calculates density, pressure, and temperature based on altitude.

ISA (国际标准大气) 模型。
根据高度计算密度、压强和温度。
"""
import numpy as np
from modules.utils.constants import GRAVITY

class Atmosphere:
    RHO_0 = 1.225; P_0 = 101325.0; T_0 = 288.15
    L = 0.0065; R = 287.05

    @staticmethod
    def get_properties(altitude):
        h = np.clip(altitude, 0, 11000)
        T = Atmosphere.T_0 - (Atmosphere.L * h)
        exponent = GRAVITY / (Atmosphere.L * Atmosphere.R)
        P = Atmosphere.P_0 * (1 - (Atmosphere.L * h) / Atmosphere.T_0) ** exponent
        rho = P / (Atmosphere.R * T)
        a = np.sqrt(1.4 * Atmosphere.R * T)
        return rho, T, P, a
```

---

### File: `modules/environment/turbulence.py`

```python
"""
Dryden Turbulence Model.
Generates random wind gusts matching atmospheric power spectral density.

Dryden 湍流模型。
生成符合大气功率谱密度的随机阵风。
"""
import numpy as np


class DrydenTurbulence:
    def __init__(self, intensity=None):
        self.u_prev = 0.0;
        self.v_prev = 0.0;
        self.w_prev = 0.0
        if intensity is None: intensity = [1.0, 1.0, 1.0]
        self.sigma_u = intensity[0]
        self.sigma_v = intensity[1]
        self.sigma_w = intensity[2]

    def update(self, altitude, airspeed, dt):
        V = max(airspeed, 10.0)
        # Scale lengths (Low altitude model) / 尺度长度 (低空模型)
        L_u = V / 0.5;
        L_v = V / 0.5
        L_w = altitude if altitude < 300 else 300.0

        noise = np.random.normal(0, 1, 3)

        # Band-limited white noise filters / 带限白噪声滤波器
        T_u = L_u / V;
        coeff_u = np.sqrt(2 * self.sigma_u ** 2 * dt / T_u)
        self.u_prev = (1 - dt / T_u) * self.u_prev + coeff_u * noise[0]

        T_v = L_v / V;
        coeff_v = np.sqrt(2 * self.sigma_v ** 2 * dt / T_v)
        self.v_prev = (1 - dt / T_v) * self.v_prev + coeff_v * noise[1]

        T_w = L_w / V;
        coeff_w = np.sqrt(2 * self.sigma_w ** 2 * dt / T_w)
        self.w_prev = (1 - dt / T_w) * self.w_prev + coeff_w * noise[2]

        return np.array([self.u_prev, self.v_prev, self.w_prev])
```

---

### File: `modules/control/__init__.py`

```python
"""
Flight Control System.
Contains PID algorithms, Autopilot logic, and Control Mixers.

飞行控制系统。
包含 PID 算法、自动驾驶逻辑和混控器。
"""
```

---

### File: `modules/control/autopilot.py`

```python
"""
Autopilot Core Logic.
Implements Advanced Control Laws:
1. Vertical Speed Mode (Altitude -> Climb Rate -> Pitch).
2. Yaw Damper (Yaw Rate -> Rudder).
3. Gain Scheduling.

自动驾驶仪核心逻辑。
实现高级控制律：
1. 垂直速度模式 (高度 -> 爬升率 -> 俯仰)。
2. 偏航阻尼器 (偏航角速度 -> 方向舵)。
3. 增益调度。
"""

import numpy as np
from modules.control.pid import PID
from modules.utils.math3d import MathUtils

class Autopilot:
    def __init__(self, config):
        self.design_speed = config.get('design_speed', 60.0)
        self.limits = config.get('limits', {
            'max_climb_rate': 5.0, 'max_sink_rate': 5.0,
            'max_bank': 0.52, 'max_pitch': 0.35
        })

        # --- Longitudinal Loops / 纵向回路 ---
        # 1. Alt -> Climb Rate
        a = config['altitude_hold']
        self.alt_pid = PID(a['kp'], a['ki'], a['kd'], a['out_min'], a['out_max'])

        # 2. Climb Rate -> Pitch [NEW]
        c = config.get('climb_rate_hold', {'kp':0.1, 'ki':0, 'kd':0, 'out_min':-0.3, 'out_max':0.3})
        self.vs_pid = PID(c['kp'], c['ki'], c['kd'], c['out_min'], c['out_max'])

        # 3. Pitch -> Elevator
        p = config['pitch_hold']
        self.pitch_pid = PID(p['kp'], p['ki'], p['kd'], p['out_min'], p['out_max'])

        # --- Lateral Loops / 横侧向回路 ---
        h = config['heading_hold']
        self.hdg_pid = PID(h['kp'], h['ki'], h['kd'], h['out_min'], h['out_max'])

        r = config['roll_hold']
        self.roll_pid = PID(r['kp'], r['ki'], r['kd'], r['out_min'], r['out_max'])

        # Yaw Damper [NEW]
        y = config.get('yaw_damper', {'kp':0, 'ki':0, 'kd':0, 'out_min':0, 'out_max':0})
        self.yaw_damp_pid = PID(y['kp'], y['ki'], y['kd'], y['out_min'], y['out_max'])

        # --- Speed / 速度 ---
        s = config['speed_hold']
        self.speed_pid = PID(s['kp'], s['ki'], s['kd'], s['out_min'], s['out_max'])

        self.target_alt = 2000.0
        self.target_spd = 60.0
        self.target_hdg = 0.0

    def set_targets(self, alt, speed, heading_deg=0.0):
        self.target_alt = alt
        self.target_spd = speed
        self.target_hdg = np.radians(heading_deg)

    def _wrap_angle(self, angle):
        """
        Keep angle between -pi and pi.
        保持角度在 -pi 到 pi 之间。
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def update(self, imu, air_data, dt):
        # Gain Scheduling / 增益调度
        current_speed = max(air_data.airspeed_tas, 10.0)
        scaling = (self.design_speed / current_speed) ** 2
        scaling = np.clip(scaling, 0.2, 5.0)

        # =========================================================
        # 1. Longitudinal Control (Vertical Speed Mode)
        #    纵向控制 (垂直速度模式)
        # =========================================================

        # Step A: Altitude -> Target Climb Rate
        # Limit max climb/sink rate to prevent G-load spikes
        # 限制最大爬升/下降率，防止过载尖峰
        target_vs = self.alt_pid.update(self.target_alt, air_data.altitude_baro, dt)
        target_vs = np.clip(target_vs, -self.limits['max_sink_rate'], self.limits['max_climb_rate'])

        # Step B: Climb Rate -> Target Pitch
        tgt_pitch = self.vs_pid.update(target_vs, air_data.climb_rate, dt, scale=scaling)

        # Step C: Pitch -> Elevator (Quaternion based)
        # Construct a "Pitch Only" target quaternion to measure pitch error
        # 构建“仅俯仰”目标四元数以测量俯仰误差
        q_tgt_pitch = MathUtils.euler_to_quat(0.0, tgt_pitch, imu.euler[2])
        att_err = MathUtils.get_body_frame_error(imu.quat, q_tgt_pitch)
        pitch_err = att_err[1]

        pitch_cmd = self.pitch_pid.update(pitch_err, 0.0, dt, scale=scaling)

        # =========================================================
        # 2. Lateral Control (Heading & Yaw Damper)
        #    横侧向控制 (航向保持 & 偏航阻尼)
        # =========================================================

        # Step A: Heading -> Target Roll
        hdg_error = self._wrap_angle(self.target_hdg - imu.euler[2])
        tgt_roll = self.hdg_pid.update(hdg_error, 0.0, dt)
        tgt_roll = np.clip(tgt_roll, -self.limits['max_bank'], self.limits['max_bank'])

        # Step B: Roll -> Aileron
        # Construct "Roll Only" target (using current pitch/yaw)
        # 构建“仅滚转”目标
        q_tgt_roll = MathUtils.euler_to_quat(tgt_roll, imu.euler[1], imu.euler[2])
        att_err_roll = MathUtils.get_body_frame_error(imu.quat, q_tgt_roll)
        roll_err = att_err_roll[0]

        roll_cmd = self.roll_pid.update(roll_err, 0.0, dt, scale=scaling)

        # Step C: Yaw Damper (Yaw Rate -> Rudder)
        # Target Yaw Rate is 0 (stop oscillation)
        # Rudder opposes Yaw Rate (Negative feedback)
        # 目标偏航角速度为 0 (停止震荡)。方向舵抵抗偏航角速度 (负反馈)。
        yaw_rate = imu.angular_rates[2]
        rud_cmd = self.yaw_damp_pid.update(0.0, yaw_rate, dt, scale=scaling)

        # =========================================================
        # 3. Speed Control
        #    速度控制
        # =========================================================
        thr_cmd = self.speed_pid.update(self.target_spd, air_data.airspeed_tas, dt)

        return pitch_cmd, roll_cmd, rud_cmd, thr_cmd
```

---

### File: `modules/control/mixer.py`

```python
"""
Control Mixer.
Maps logical commands (Pitch/Roll/Yaw Cmd) to physical actuators (Elevator/Aileron/Rudder).
Supports Standard, Delta Wing, and V-Tail configurations.

控制混控器。
将逻辑指令映射到物理舵面。
支持常规布局、三角翼和 V 尾布局。
"""
from modules.dynamics.forces import ControlInputs


class Mixer:
    def __init__(self, mixer_type="standard"):
        self.type = mixer_type

    def mix(self, pitch_cmd, roll_cmd, yaw_cmd, throttle_cmd):
        controls = ControlInputs()
        controls.throttle = throttle_cmd

        if self.type == "standard":
            controls.elevator = pitch_cmd
            controls.aileron = roll_cmd
            controls.rudder = yaw_cmd
        elif self.type == "delta":
            controls.elevator = pitch_cmd + roll_cmd
            controls.aileron = pitch_cmd - roll_cmd
            controls.rudder = yaw_cmd
        elif self.type == "v_tail":
            controls.elevator = pitch_cmd + yaw_cmd
            controls.rudder = pitch_cmd - yaw_cmd
            controls.aileron = roll_cmd

        return controls
```

---

### File: `modules/control/pid.py`

```python
"""
Generic PID Controller implementation.
Supports integral windup protection and gain scaling.

通用 PID 控制器实现。
支持积分抗饱和与增益缩放。
"""
import numpy as np


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp;
        self.ki = ki;
        self.kd = kd
        self.out_min = out_min;
        self.out_max = out_max
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_run = True

    def update(self, setpoint, measurement, dt, scale=1.0):
        error = setpoint - measurement
        eff_kp = self.kp * scale
        eff_ki = self.ki * scale
        eff_kd = self.kd * scale

        P = eff_kp * error
        self.integral += error * dt
        I = eff_ki * self.integral

        if self.first_run:
            derivative = 0.0; self.first_run = False
        else:
            derivative = (error - self.prev_error) / dt
        D = eff_kd * derivative
        self.prev_error = error

        return np.clip(P + I + D, self.out_min, self.out_max)
```

---
