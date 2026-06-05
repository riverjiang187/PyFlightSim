"""
ISA (International Standard Atmosphere) Model.
Calculates density, pressure, and temperature based on altitude.
Supports Troposphere (0-11km) and Lower Stratosphere (11-20km).

ISA (国际标准大气) 模型。
根据高度计算密度、压强和温度。
支持对流层 (0-11km) 和平流层下部 (11-20km)。
"""
import numpy as np
from modules.utils.constants import GRAVITY

class Atmosphere:
    # Sea Level Constants / 海平面常数
    RHO_0 = 1.225       # kg/m^3
    P_0 = 101325.0      # Pa
    T_0 = 288.15        # K

    # Gas Constants / 气体常数
    R = 287.05          # J/(kg*K)
    GAMMA = 1.4         # Specific heat ratio

    # Troposphere (0 - 11,000m) / 对流层
    H_TROP = 11000.0    # m
    L_TROP = 0.0065     # K/m (Lapse rate)

    # Stratosphere (11,000m - 20,000m) / 平流层下部
    T_STRAT = 216.65    # K (-56.5 deg C, constant temp)
    P_STRAT = 22632.1   # Pa (Pressure at 11km)

    @staticmethod
    def get_properties(altitude):
        """
        Returns: (density, temperature, pressure, speed_of_sound)
        """
        # Clamp altitude to max supported (20km) to prevent math errors
        # 限制最大高度为 20km，防止数学错误
        h = np.clip(altitude, 0.0, 20000.0)

        if h < Atmosphere.H_TROP:
            # --- Troposphere (0 to 11km) ---
            T = Atmosphere.T_0 - (Atmosphere.L_TROP * h)

            exponent = GRAVITY / (Atmosphere.L_TROP * Atmosphere.R)
            P = Atmosphere.P_0 * (1.0 - (Atmosphere.L_TROP * h) / Atmosphere.T_0) ** exponent

        else:
            # --- Lower Stratosphere (11km to 20km) ---
            T = Atmosphere.T_STRAT

            # Isothermal pressure equation: P = P11 * exp(-g * (h - h11) / (R * T))
            # 等温压强公式
            dh = h - Atmosphere.H_TROP
            P = Atmosphere.P_STRAT * np.exp(-GRAVITY * dh / (Atmosphere.R * Atmosphere.T_STRAT))

        # Density (Ideal Gas Law: rho = P / RT)
        rho = P / (Atmosphere.R * T)

        # Speed of Sound (a = sqrt(gamma * R * T))
        a = np.sqrt(Atmosphere.GAMMA * Atmosphere.R * T)

        return rho, T, P, a