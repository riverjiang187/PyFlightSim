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