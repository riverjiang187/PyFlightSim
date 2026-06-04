"""
Aerodynamics and Thrust calculation module.
Computes Lift, Drag, and Moments based on Stability Derivatives.
Includes Non-Linear Stall Model using Sigmoid Blending.

气动力与推力计算模块。
基于稳定性导数计算升力、阻力和力矩。
包含基于 Sigmoid 混合函数的非线性失速模型。
"""
import numpy as np
from dataclasses import dataclass

@dataclass
class AeroParams:
    S: float; b: float; c: float
    C_L_0: float; C_L_alpha: float
    C_D_0: float; K: float
    C_m_0: float; C_m_alpha: float; C_m_q: float
    C_l_delta_a: float = 0.1
    C_n_delta_r: float = 0.05
    C_n_beta: float = 0.1
    C_l_p: float = -0.4
    C_n_r: float = -0.2
    alpha_stall: float = 0.26
    stall_width: float = 0.05

    # --- FIX: Added max_thrust to decouple propulsion from code ---
    # 修复：添加 max_thrust 以将动力系统与代码解耦
    max_thrust: float = 5000.0 # Default fallback value

@dataclass
class ControlInputs:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0

class Aerodynamics:
    def __init__(self, params: AeroParams):
        self.params = params

    def _sigmoid(self, x, center, width):
        return 1.0 / (1.0 + np.exp(-4.0 * (abs(x) - center) / width))

    def get_forces_and_moments(self, state, density, controls, wind_body_vector=np.zeros(3)):
        v_air_vec = state.vel - wind_body_vector
        u, v, w = v_air_vec
        V_sq = u**2 + v**2 + w**2
        V_tas = np.sqrt(V_sq)

        if V_tas < 0.1: return np.zeros(3), np.zeros(3)

        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / V_tas, -1, 1))
        q_bar = 0.5 * density * V_sq

        # --- Aerodynamic Coefficients ---
        cl_linear = self.params.C_L_0 + self.params.C_L_alpha * alpha
        cd_linear = self.params.C_D_0 + self.params.K * (cl_linear**2)

        cl_stall = 1.0 * np.sin(2 * alpha)
        cd_stall = 2.0 * np.sin(alpha)**2 + self.params.C_D_0

        sigma = self._sigmoid(alpha, self.params.alpha_stall, self.params.stall_width)

        C_L = (1 - sigma) * cl_linear + sigma * cl_stall
        C_D = (1 - sigma) * cd_linear + sigma * cd_stall

        elevator_eff = (1 - 0.5 * sigma)
        C_L += (0.5 * controls.elevator) * elevator_eff

        q_hat = (self.params.c * state.rates[1]) / (2 * V_tas)
        C_m = (self.params.C_m_0 + self.params.C_m_alpha * alpha +
               self.params.C_m_q * q_hat + -1.5 * controls.elevator * elevator_eff)

        p_hat = (self.params.b * state.rates[0]) / (2 * V_tas)
        r_hat = (self.params.b * state.rates[2]) / (2 * V_tas)

        C_l = (self.params.C_l_delta_a * controls.aileron) + (self.params.C_l_p * p_hat)
        C_n = (self.params.C_n_delta_r * controls.rudder) + \
              (self.params.C_n_beta * beta) + (self.params.C_n_r * r_hat)

        # --- Forces & Moments ---
        L = q_bar * self.params.S * C_L
        D = q_bar * self.params.S * C_D
        Fx = -D * np.cos(alpha) + L * np.sin(alpha)
        Fz = -D * np.sin(alpha) - L * np.cos(alpha)

        # --- FIX: Use configured max_thrust instead of hardcoded value ---
        # 修复：使用配置的最大推力，而不是硬编码的值
        Fx += self.params.max_thrust * controls.throttle

        Roll = q_bar * self.params.S * self.params.b * C_l
        Pitch = q_bar * self.params.S * self.params.c * C_m
        Yaw = q_bar * self.params.S * self.params.b * C_n

        return np.array([Fx, 0.0, Fz]), np.array([Roll, Pitch, Yaw])