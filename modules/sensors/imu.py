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
    def __init__(self, config=None, enable_noise=False):
        self.data = IMUData()
        self.enable_noise = enable_noise
        
        # Parse config or use defaults
        if config is None:
            config = {}
            
        self.gyro_noise_std = config.get('gyro_noise_std', 0.0)
        self.gyro_bias_tau = config.get('gyro_bias_tau', 100.0)
        self.gyro_bias_noise_std = config.get('gyro_bias_noise_std', 0.0)
        
        self.accel_noise_std = config.get('accel_noise_std', 0.0)
        self.accel_bias_tau = config.get('accel_bias_tau', 100.0)
        self.accel_bias_noise_std = config.get('accel_bias_noise_std', 0.0)
        
        # State variables for bias
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)

    def update(self, state, forces_body_nongrav, mass, dt=0.01):
        # True values
        true_angular_rates = state.rates.copy()
        true_specific_force = forces_body_nongrav / mass if mass > 0 else np.zeros(3)

        if self.enable_noise:
            # 1. Update biases (Gauss-Markov process)
            # b_dot = -1/tau * b + noise
            # b(t+dt) = b(t) + dt * (-1/tau * b(t) + noise)
            gyro_b_dot = - (1.0 / max(self.gyro_bias_tau, 1e-5)) * self.gyro_bias + np.random.normal(0, self.gyro_bias_noise_std, 3)
            self.gyro_bias += gyro_b_dot * dt
            
            accel_b_dot = - (1.0 / max(self.accel_bias_tau, 1e-5)) * self.accel_bias + np.random.normal(0, self.accel_bias_noise_std, 3)
            self.accel_bias += accel_b_dot * dt

            # 2. Add bias and white noise to output
            # y = x_true + b + eta
            self.data.angular_rates = true_angular_rates + self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)
            self.data.specific_force = true_specific_force + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
        else:
            self.data.angular_rates = true_angular_rates
            self.data.specific_force = true_specific_force

        # Output both Euler (for logging) and Quat (for control)
        # 姿态通常由后续纯惯导或互补滤波得到，此处暂透传真值，可在此阶段叠加观测噪声
        self.data.euler = state.euler_angles
        self.data.quat = state.q.copy()

    def get_data(self): return self.data