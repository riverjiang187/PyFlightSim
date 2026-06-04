"""
Dryden Turbulence Model.
Generates random wind gusts matching atmospheric power spectral density.
Uses Exact Discretization to guarantee unconditional mathematical stability.

Dryden 湍流模型。
生成符合大气功率谱密度的随机阵风。
使用精确离散化方法以保证无条件的数学稳定性。
"""
import numpy as np

class DrydenTurbulence:
    def __init__(self, intensity=None):
        self.u_prev = 0.0
        self.v_prev = 0.0
        self.w_prev = 0.0

        if intensity is None:
            intensity = [1.0, 1.0, 1.0]

        self.sigma_u = intensity[0]
        self.sigma_v = intensity[1]
        self.sigma_w = intensity[2]

    def update(self, altitude, airspeed, dt):
        # Prevent division by zero for airspeed
        # 防止空速为零导致除以零
        V = max(airspeed, 1.0)

        # Clamp altitude to avoid negative values if aircraft crashes underground
        # 限制高度下限，防止飞机钻地后产生负高度
        h_safe = max(altitude, 10.0)

        # 1. Calculate Turbulence Scale Lengths (L)
        #    计算湍流尺度长度
        # Low altitude model approximation
        L_u = h_safe / (0.177 + 0.000823 * h_safe)**1.2
        L_v = L_u
        L_w = h_safe

        # Cap scale lengths to high-altitude values if above 1000ft (~300m)
        if h_safe >= 300.0:
            L_u = 533.0
            L_v = 533.0
            L_w = 533.0

        # 2. Calculate Time Constants (T = L / V)
        #    计算时间常数
        T_u = L_u / V
        T_v = L_v / V
        T_w = L_w / V

        # Generate white noise
        noise = np.random.normal(0, 1, 3)

        # --- FIX: Exact Discretization for Unconditional Stability ---
        # 修复：使用精确离散化以保证无条件稳定
        # Formula: x[k+1] = exp(-dt/T) * x[k] + sqrt(1 - exp(-2*dt/T)) * sigma * noise

        # U-axis (Longitudinal)
        decay_u = np.exp(-dt / T_u)
        gain_u = np.sqrt(1.0 - np.exp(-2.0 * dt / T_u)) * self.sigma_u
        self.u_prev = decay_u * self.u_prev + gain_u * noise[0]

        # V-axis (Lateral)
        decay_v = np.exp(-dt / T_v)
        gain_v = np.sqrt(1.0 - np.exp(-2.0 * dt / T_v)) * self.sigma_v
        self.v_prev = decay_v * self.v_prev + gain_v * noise[1]

        # W-axis (Vertical)
        decay_w = np.exp(-dt / T_w)
        gain_w = np.sqrt(1.0 - np.exp(-2.0 * dt / T_w)) * self.sigma_w
        self.w_prev = decay_w * self.w_prev + gain_w * noise[2]

        return np.array([self.u_prev, self.v_prev, self.w_prev])