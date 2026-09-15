"""
Global Positioning System (GPS).
Converts local NED coordinates to Geodetic (Lat/Lon/Alt).
Includes singularity protection near the poles.

全球定位系统 (GPS)。
将局部 NED 坐标转换为大地坐标 (经纬度)。
包含极点附近的奇异性保护。
"""
import copy
from collections import deque
from dataclasses import dataclass
from modules.utils.math3d import MathUtils
import numpy as np

@dataclass
class GPSReading:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    ground_speed: float = 0.0
    ground_course: float = 0.0

class GPS:
    def __init__(self, home_lat=37.7749, home_lon=-122.4194, config=None, enable_noise=False):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.reading = GPSReading()
        self.R_EARTH = 6378137.0
        self.DEG_TO_RAD = np.pi / 180.0
        self.RAD_TO_DEG = 180.0 / np.pi
        
        self.enable_noise = enable_noise
        if config is None: config = {}
        
        self.update_rate = config.get('update_rate', 10.0)
        self.delay = config.get('delay', 0.15)
        self.horizontal_std = config.get('horizontal_std', 1.5)
        self.vertical_std = config.get('vertical_std', 3.0)
        
        self.update_period = 1.0 / max(self.update_rate, 0.1)
        self.time_since_last_update = self.update_period  # Trigger immediately on first step
        
        self.delay_buffer = deque()
        self.current_time = 0.0

    def update(self, state, dt=0.01):
        self.current_time += dt
        self.time_since_last_update += dt
        
        # 1. ZOH Update Step
        if self.time_since_last_update >= self.update_period - 1e-5:
            # Subtract instead of modulo to avoid floating point math issues
            self.time_since_last_update -= self.update_period
            
            new_reading = GPSReading()
            
            # Noise generation
            n_err = np.random.normal(0, self.horizontal_std) if self.enable_noise else 0.0
            e_err = np.random.normal(0, self.horizontal_std) if self.enable_noise else 0.0
            alt_err = np.random.normal(0, self.vertical_std) if self.enable_noise else 0.0
            
            # Latitude
            d_lat_rad = (state.pos[0] + n_err) / self.R_EARTH
            new_reading.latitude = self.home_lat + (d_lat_rad * self.RAD_TO_DEG)

            # Longitude
            safe_lat = np.clip(self.home_lat, -89.9, 89.9)
            scale = np.cos(safe_lat * self.DEG_TO_RAD)
            d_lon_rad = (state.pos[1] + e_err) / (self.R_EARTH * scale)
            new_reading.longitude = self.home_lon + (d_lon_rad * self.RAD_TO_DEG)

            # Altitude
            new_reading.altitude = -state.pos[2] + alt_err

            # Ground Speed & Course
            R_b_n = MathUtils.quat_to_rotation_matrix(state.q)
            vel_ned = R_b_n @ state.vel
            vn, ve = vel_ned[0], vel_ned[1]
            new_reading.ground_speed = np.sqrt(vn**2 + ve**2)
            new_reading.ground_course = np.degrees(np.arctan2(ve, vn)) % 360.0
            
            self.delay_buffer.append((self.current_time, new_reading))

        # 2. Output & Delay logic
        if self.enable_noise and self.delay > 0:
            # Advance the buffer until the next element is NOT older than the delay
            while len(self.delay_buffer) > 1 and (self.current_time - self.delay_buffer[1][0]) >= self.delay:
                self.delay_buffer.popleft()
                
            # If the oldest element is older than delay, output it
            if len(self.delay_buffer) > 0 and (self.current_time - self.delay_buffer[0][0]) >= self.delay:
                self.reading = copy.deepcopy(self.delay_buffer[0][1])
            elif self.reading.latitude == 0.0 and len(self.delay_buffer) > 0:
                # Initialization fallback
                self.reading = copy.deepcopy(self.delay_buffer[0][1])
        else:
            # Real-time output
            if len(self.delay_buffer) > 0:
                self.reading = copy.deepcopy(self.delay_buffer[-1][1])
                self.delay_buffer.clear()

    def get_reading(self):
        return self.reading