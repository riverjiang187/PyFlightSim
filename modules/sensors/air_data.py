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
    def __init__(self, config=None, enable_noise=False):
        self.reading = AirDataReading()
        self.enable_noise = enable_noise
        
        if config is None: config = {}
        self.airspeed_noise_std = config.get('airspeed_noise_std', 0.5)
        self.alpha_noise_std = config.get('alpha_noise_std', 0.01)
        self.beta_noise_std = config.get('beta_noise_std', 0.01)
        self.altitude_noise_std = config.get('altitude_noise_std', 1.0)
        self.climb_rate_noise_std = config.get('climb_rate_noise_std', 0.2)

    def update(self, state, wind_body: np.ndarray = np.zeros(3)) -> None:
        # 1. Flow Angles & Airspeed / 气流角与空速
        v_air_vec = state.vel - wind_body
        u, v, w = v_air_vec
        V_sq = u**2 + v**2 + w**2
        V_tas = np.sqrt(V_sq)
        
        # Noise components
        v_noise = np.random.normal(0, self.airspeed_noise_std) if self.enable_noise else 0.0
        alpha_noise = np.random.normal(0, self.alpha_noise_std) if self.enable_noise else 0.0
        beta_noise = np.random.normal(0, self.beta_noise_std) if self.enable_noise else 0.0
        alt_noise = np.random.normal(0, self.altitude_noise_std) if self.enable_noise else 0.0
        climb_noise = np.random.normal(0, self.climb_rate_noise_std) if self.enable_noise else 0.0

        self.reading.airspeed_tas = V_tas + v_noise

        if V_tas > 0.1:
            self.reading.alpha = np.arctan2(w, u) + alpha_noise
            self.reading.beta = np.arcsin(np.clip(v / V_tas, -1.0, 1.0)) + beta_noise
        else:
            self.reading.alpha = 0.0 + alpha_noise
            self.reading.beta = 0.0 + beta_noise

        # 2. Altitude / 高度
        self.reading.altitude_baro = -state.pos[2] + alt_noise

        # 3. Climb Rate / 爬升率
        R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
        vel_ned = R_b_n @ state.vel
        self.reading.climb_rate = -vel_ned[2] + climb_noise

    def get_reading(self): return self.reading