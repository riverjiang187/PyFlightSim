"""
Aerodynamics and Thrust calculation module.
Computes Lift, Drag, and Moments based on Stability Derivatives.
Includes Non-Linear Stall Model using Sigmoid Blending.
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

        # --- Coefficients ---

        # 1. Linear Model
        cl_linear = self.params.C_L_0 + self.params.C_L_alpha * alpha
        cd_linear = self.params.C_D_0 + self.params.K * (cl_linear**2)
        # Linear Pitch Moment
        cm_linear = self.params.C_m_0 + self.params.C_m_alpha * alpha

        # 2. Stall Model (High Alpha)
        # Lift drops, Drag spikes
        cl_stall = 0.8 * np.sin(2 * alpha)
        cd_stall = 1.8 * np.sin(alpha)**2 + self.params.C_D_0

        # [CRITICAL FIX] Deep Stall Restoring Moment
        # When Alpha > 60 deg, the aircraft naturally wants to nose down (stable).
        # We model this as a strong negative Cm proportional to sin(alpha).
        # 关键修复：深失速恢复力矩。当攻角过大时，产生自然的低头力矩。
        cm_stall = -0.8 * np.sin(alpha)

        # 3. Blending
        sigma = self._sigmoid(alpha, self.params.alpha_stall, self.params.stall_width)

        C_L = (1 - sigma) * cl_linear + sigma * cl_stall
        C_D = (1 - sigma) * cd_linear + sigma * cd_stall

        # Blend Pitch Moment
        C_m_static = (1 - sigma) * cm_linear + sigma * cm_stall

        # 4. Add Damping & Controls
        # Elevator effectiveness drops at high alpha
        elevator_eff = (1 - 0.6 * sigma)

        q_hat = (self.params.c * state.rates[1]) / (2 * V_tas)

        # Total Pitch Moment
        C_m = C_m_static + (self.params.C_m_q * q_hat) + (-1.2 * controls.elevator * elevator_eff)

        # Lateral
        p_hat = (self.params.b * state.rates[0]) / (2 * V_tas)
        r_hat = (self.params.b * state.rates[2]) / (2 * V_tas)

        C_l = (self.params.C_l_delta_a * controls.aileron) + (self.params.C_l_p * p_hat)
        C_n = (self.params.C_n_delta_r * controls.rudder) + \
              (self.params.C_n_beta * beta) + (self.params.C_n_r * r_hat)

        # 5. Final Forces & Moments
        L = q_bar * self.params.S * C_L
        D = q_bar * self.params.S * C_D

        Fx = -D * np.cos(alpha) + L * np.sin(alpha)
        Fz = -D * np.sin(alpha) - L * np.cos(alpha)

        max_thrust = 122600.0 * 2 # Su-27 has 2 engines! (~245kN total with AB)
        Fx += max_thrust * controls.throttle

        Roll = q_bar * self.params.S * self.params.b * C_l
        Pitch = q_bar * self.params.S * self.params.c * C_m
        Yaw = q_bar * self.params.S * self.params.b * C_n

        return np.array([Fx, 0.0, Fz]), np.array([Roll, Pitch, Yaw])