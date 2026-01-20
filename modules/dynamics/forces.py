"""
Aerodynamics and Thrust calculation module.
Computes Lift, Drag, and Moments based on Stability Derivatives.

气动力与推力计算模块。
基于稳定性导数计算升力、阻力和力矩。
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class AeroParams:
    S: float;
    b: float;
    c: float
    C_L_0: float;
    C_L_alpha: float
    C_D_0: float;
    K: float
    C_m_0: float;
    C_m_alpha: float;
    C_m_q: float
    C_l_delta_a: float = 0.1
    C_n_delta_r: float = 0.05
    C_n_beta: float = 0.1
    C_l_p: float = -0.4
    C_n_r: float = -0.2


@dataclass
class ControlInputs:
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    throttle: float = 0.0


class Aerodynamics:
    def __init__(self, params: AeroParams):
        self.params = params

    def get_forces_and_moments(self, state, density, controls, wind_body_vector=np.zeros(3)):
        # Calculate Airspeed (Body Vel - Wind Vel)
        # 计算空速 (机体速度 - 风速)
        v_air_vec = state.vel - wind_body_vector
        u, v, w = v_air_vec
        V_sq = u ** 2 + v ** 2 + w ** 2
        V_tas = np.sqrt(V_sq)

        if V_tas < 0.1: return np.zeros(3), np.zeros(3)

        # Alpha & Beta / 攻角与侧滑角
        alpha = np.arctan2(w, u)
        beta = np.arcsin(np.clip(v / V_tas, -1, 1))
        q_bar = 0.5 * density * V_sq

        # Longitudinal Coeffs / 纵向系数
        C_L = self.params.C_L_0 + self.params.C_L_alpha * alpha + (0.5 * controls.elevator)
        C_D = self.params.C_D_0 + self.params.K * (C_L ** 2)
        q_hat = (self.params.c * state.rates[1]) / (2 * V_tas)
        C_m = (self.params.C_m_0 + self.params.C_m_alpha * alpha +
               self.params.C_m_q * q_hat + -1.5 * controls.elevator)

        # Lateral Coeffs / 横侧向系数
        p_hat = (self.params.b * state.rates[0]) / (2 * V_tas)
        r_hat = (self.params.b * state.rates[2]) / (2 * V_tas)

        C_l = (self.params.C_l_delta_a * controls.aileron) + (self.params.C_l_p * p_hat)
        C_n = (self.params.C_n_delta_r * controls.rudder) + \
              (self.params.C_n_beta * beta) + (self.params.C_n_r * r_hat)

        # Forces (Stability -> Body Frame) / 力 (稳定性坐标系 -> 机体坐标系)
        L = q_bar * self.params.S * C_L
        D = q_bar * self.params.S * C_D
        Fx = -D * np.cos(alpha) + L * np.sin(alpha)
        Fz = -D * np.sin(alpha) - L * np.cos(alpha)

        # Thrust / 推力
        max_thrust = 5000.0
        Fx += max_thrust * controls.throttle

        # Moments / 力矩
        Roll = q_bar * self.params.S * self.params.b * C_l
        Pitch = q_bar * self.params.S * self.params.c * C_m
        Yaw = q_bar * self.params.S * self.params.b * C_n

        return np.array([Fx, 0.0, Fz]), np.array([Roll, Pitch, Yaw])