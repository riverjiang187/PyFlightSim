"""
Calculates Rigid Body Equations of Motion (EOM).
Includes Translational (Newton) and Rotational (Euler) equations.

计算刚体动力学微分方程。
包含平动 (牛顿) 和转动 (欧拉) 方程。
"""
import numpy as np
from modules.utils.math3d import MathUtils
from modules.utils.constants import GRAVITY


class MassProperties:
    def __init__(self, mass, Ixx, Iyy, Izz):
        self.mass = mass
        self.inertia = np.array([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])
        self.inertia_inv = np.linalg.inv(self.inertia)


class Kinematics:
    def __init__(self, mass_props):
        self.mass_props = mass_props

    def get_state_derivative(self, state, forces_body, moments_body):
        """
        Calculates dx/dt given current state x and inputs u (forces).
        根据当前状态和输入力计算状态导数。
        """
        vel_b = state.vel
        q = state.q
        omega_b = state.rates

        # 1. Kinematics (Pos & Att) / 运动学 (位置与姿态)
        R_b_n = MathUtils.quat_to_rotation_matrix(q)
        pos_dot = R_b_n @ vel_b
        quat_dot = MathUtils.quat_derivative(q, omega_b)

        # 2. Dynamics (Vel & Rates) / 动力学 (速度与角速度)
        # Gravity in Body Frame / 机体坐标系下的重力
        g_ned = np.array([0, 0, GRAVITY])
        g_body = R_b_n.T @ g_ned

        # F = ma -> a = F/m
        F_total_b = forces_body + (g_body * self.mass_props.mass)
        coriolis = np.cross(omega_b, vel_b)
        vel_dot = (F_total_b / self.mass_props.mass) - coriolis

        # M = I*alpha + w x I*w
        J = self.mass_props.inertia
        J_inv = self.mass_props.inertia_inv
        gyroscopic = np.cross(omega_b, (J @ omega_b))
        rates_dot = J_inv @ (moments_body - gyroscopic)

        return np.concatenate([pos_dot, vel_dot, quat_dot, rates_dot])