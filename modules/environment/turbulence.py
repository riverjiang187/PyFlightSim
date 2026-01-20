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