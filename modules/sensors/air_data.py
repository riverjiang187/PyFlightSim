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