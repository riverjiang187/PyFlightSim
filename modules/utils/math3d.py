"""
3D Math Utility Library.
Handles Quaternion <-> Euler conversions and Rotation Matrix (DCM) calculations.
Includes quaternion algebra for control error computation.

3D 数学工具库。
主要处理四元数与欧拉角的转换，以及坐标系旋转矩阵的计算。
包含用于计算控制误差的四元数代数运算。

Convention / 约定:
- Quaternions: Scalar-first [w, x, y, z] / 实部在前
- Euler: [Roll, Pitch, Yaw] (Radians) / 弧度制
"""

import numpy as np

class MathUtils:
    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        """
        Converts Euler angles to Quaternion [w, x, y, z].
        将欧拉角转换为四元数。
        """
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ])

    @staticmethod
    def quat_to_euler(q):
        """
        Converts Quaternion to Euler angles [roll, pitch, yaw].
        将四元数转换为欧拉角。
        """
        q0, q1, q2, q3 = q
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q0 * q2 - q3 * q1)
        if np.abs(sinp) >= 1: pitch = np.copysign(np.pi / 2, sinp)
        else: pitch = np.arcsin(sinp)

        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return np.array([roll, pitch, yaw])

    @staticmethod
    def normalize_quat(q):
        """
        Ensures quaternion is unit length.
        确保四元数为单位长度。
        """
        norm = np.linalg.norm(q)
        return q / norm if norm > 0 else np.array([1.0, 0.0, 0.0, 0.0])

    @staticmethod
    def quat_to_rotation_matrix(q):
        """
        Calculates Rotation Matrix (Body -> NED) from Quaternion.
        计算从机体坐标系到地面坐标系的旋转矩阵。
        """
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2*(q2**2 + q3**2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2)],
            [2*(q1*q2 + q0*q3),       1 - 2*(q1**2 + q3**2),   2*(q2*q3 - q0*q1)],
            [2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),   1 - 2*(q1**2 + q2**2)]
        ])

    @staticmethod
    def quat_derivative(q, rates):
        """
        Calculates dq/dt = 0.5 * q * omega.
        计算四元数导数。
        """
        p, q_rate, r = rates
        omega_mat = np.array([
            [0, -p, -q_rate, -r],
            [p,  0,  r, -q_rate],
            [q_rate, -r,  0,  p],
            [r,  q_rate, -p,  0]
        ])
        return 0.5 * (omega_mat @ q)

    @staticmethod
    def quat_conjugate(q):
        """
        Returns the conjugate (inverse for unit quats) of q.
        返回四元数的共轭（对于单位四元数即为逆）。
        """
        return np.array([q[0], -q[1], -q[2], -q[3]])

    @staticmethod
    def quat_multiply(q1, q2):
        """
        Multiplies two quaternions.
        四元数乘法。
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def get_body_frame_error(q_current, q_target):
        """
        Calculates the rotation error vector in Body Frame.
        Returns [roll_err, pitch_err, yaw_err] in radians.

        计算机体坐标系下的旋转误差向量。
        返回 [滚转误差, 俯仰误差, 偏航误差] (弧度)。
        """
        # q_error = q_current_inv * q_target
        q_curr_inv = MathUtils.quat_conjugate(q_current)
        q_err = MathUtils.quat_multiply(q_curr_inv, q_target)

        # Normalize to be safe / 归一化以确保安全
        q_err = MathUtils.normalize_quat(q_err)

        # Handle double cover (q and -q are same rotation)
        # Ensure we take the shortest path
        # 处理双倍覆盖问题（q 和 -q 表示相同的旋转），确保走最短路径
        if q_err[0] < 0:
            q_err = -q_err

        # Extract vector part for small angle approximation or full conversion
        # 提取向量部分
        # Roll error (x), Pitch error (y), Yaw error (z)
        # Using 2*atan2 is robust for large angles
        return np.array([
            2.0 * np.arctan2(q_err[1], q_err[0]),
            2.0 * np.arctan2(q_err[2], q_err[0]),
            2.0 * np.arctan2(q_err[3], q_err[0])
        ])