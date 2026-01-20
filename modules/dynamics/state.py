"""
Defines the AircraftState class, the Single Source of Truth for simulation.
Contains Position, Velocity, Attitude (Quaternion), and Angular Rates.

定义 AircraftState 类，作为仿真的单一事实来源。
包含位置、速度、姿态（四元数）和角速度。
"""
import numpy as np
from modules.utils.math3d import MathUtils


class AircraftState:
    def __init__(self, pos=None, vel=None, att=None, rates=None):
        # Pos: NED Frame (m) / 位置 (北东下)
        self._pos = pos.astype(float) if pos is not None else np.zeros(3)
        # Vel: Body Frame (m/s) / 速度 (机体轴)
        self._vel = vel.astype(float) if vel is not None else np.zeros(3)
        # Att: Quaternion [w, x, y, z] / 姿态 (四元数)
        self._q = att.astype(float) if att is not None else np.array([1., 0., 0., 0.])
        # Rates: Body Frame (rad/s) / 角速度 (机体轴)
        self._rates = rates.astype(float) if rates is not None else np.zeros(3)

        self._q = MathUtils.normalize_quat(self._q)

    @property
    def pos(self): return self._pos

    @property
    def vel(self): return self._vel

    @property
    def q(self): return self._q

    @property
    def rates(self): return self._rates

    @property
    def euler_angles(self): return MathUtils.quat_to_euler(self._q)

    def to_vector(self):
        """
        Serialize state to 13-element vector for integrator.
        将状态序列化为 13 维向量以供积分器使用。
        """
        return np.concatenate([self._pos, self._vel, self._q, self._rates])

    def from_vector(self, vec):
        """
        Update state from vector.
        从向量更新状态。
        """
        self._pos = vec[0:3]
        self._vel = vec[3:6]
        self._q = MathUtils.normalize_quat(vec[6:10])
        self._rates = vec[10:13]