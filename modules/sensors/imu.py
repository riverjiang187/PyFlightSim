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