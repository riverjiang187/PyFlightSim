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

    def update(self, state, wind_body: np.ndarray = np.zeros(3)) -> None:
        # 1. Flow Angles & Airspeed / 气流角与空速
        v_air_vec = state.vel - wind_body
        u, v, w = v_air_vec
        V_sq = u**2 + v**2 + w**2
        V_tas = np.sqrt(V_sq)
        self.reading.airspeed_tas = V_tas

        if V_tas > 0.1:
            self.reading.alpha = np.arctan2(w, u)
            self.reading.beta = np.arcsin(np.clip(v / V_tas, -1.0, 1.0))
        else:
            self.reading.alpha = 0.0
            self.reading.beta = 0.0

        # 2. Altitude / 高度
        self.reading.altitude_baro = -state.pos[2]

        # 3. Climb Rate / 爬升率
        R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
        vel_ned = R_b_n @ state.vel
        self.reading.climb_rate = -vel_ned[2]

    def get_reading(self): return self.reading