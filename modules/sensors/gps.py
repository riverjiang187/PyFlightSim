"""
Global Positioning System (GPS).
Converts local NED coordinates to Geodetic (Lat/Lon/Alt).

全球定位系统 (GPS)。
将局部 NED 坐标转换为大地坐标 (经纬度)。
"""
import numpy as np
from dataclasses import dataclass
from modules.utils.math3d import MathUtils


@dataclass
class GPSReading:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    ground_speed: float = 0.0
    ground_course: float = 0.0


class GPS:
    def __init__(self, home_lat=37.7749, home_lon=-122.4194):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.reading = GPSReading()
        self.R_EARTH = 6378137.0
        self.DEG_TO_RAD = np.pi / 180.0
        self.RAD_TO_DEG = 180.0 / np.pi

    def update(self, state):
        # Flat Earth Projection / 平地投影近似
        d_lat_rad = state.pos[0] / self.R_EARTH
        self.reading.latitude = self.home_lat + (d_lat_rad * self.RAD_TO_DEG)

        scale = np.cos(self.home_lat * self.DEG_TO_RAD)
        d_lon_rad = state.pos[1] / (self.R_EARTH * scale)
        self.reading.longitude = self.home_lon + (d_lon_rad * self.RAD_TO_DEG)

        self.reading.altitude = -state.pos[2]

        # Ground Speed Calculation / 地速计算
        R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
        vel_ned = R_b_n @ state.vel
        vn, ve = vel_ned[0], vel_ned[1]
        self.reading.ground_speed = np.sqrt(vn ** 2 + ve ** 2)
        self.reading.ground_course = np.degrees(np.arctan2(ve, vn)) % 360.0

    def get_reading(self): return self.reading