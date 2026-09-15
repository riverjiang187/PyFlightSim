"""
Independent State Estimator.
Receives noisy data from IMU, GPS, and ADC and reconstructs the state for GNC.
"""
import numpy as np
from dataclasses import dataclass
from modules.sensors.imu import IMUData
from modules.sensors.gps import GPSReading
from modules.sensors.air_data import AirDataReading
import copy

@dataclass
class EstimatedState:
    imu: IMUData = None
    gps: GPSReading = None
    adc: AirDataReading = None

class Estimator:
    def __init__(self, filter_alpha=0.1):
        self.estimated = EstimatedState()
        self.raw = EstimatedState()
        self.filter_alpha = filter_alpha
        
        self.lpf_angular_rates = np.zeros(3)
        self.lpf_specific_force = np.zeros(3)
        self.is_initialized = False

    def update(self, imu_data: IMUData, gps_reading: GPSReading, adc_reading: AirDataReading, dt: float):
        # 1. Store Raw Data
        self.raw.imu = copy.deepcopy(imu_data)
        self.raw.gps = copy.deepcopy(gps_reading)
        self.raw.adc = copy.deepcopy(adc_reading)
        
        # 2. Filter logic
        if not self.is_initialized:
            self.lpf_angular_rates = np.copy(imu_data.angular_rates)
            self.lpf_specific_force = np.copy(imu_data.specific_force)
            self.is_initialized = True
        else:
            # Simple Exponential Moving Average (EMA) / Low Pass Filter for IMU
            # To account for dt variation: alpha_dt = dt / (tau + dt)
            tau = 0.1 # 100ms time constant
            alpha = dt / (tau + dt)
            self.lpf_angular_rates = alpha * imu_data.angular_rates + (1 - alpha) * self.lpf_angular_rates
            self.lpf_specific_force = alpha * imu_data.specific_force + (1 - alpha) * self.lpf_specific_force

        # 3. Assemble Estimated State
        self.estimated.imu = copy.deepcopy(imu_data)
        self.estimated.imu.angular_rates = np.copy(self.lpf_angular_rates)
        self.estimated.imu.specific_force = np.copy(self.lpf_specific_force)
        
        # Pass through GPS and ADC directly for basic version (noise in ADC/GPS is handled downstream or acceptable)
        self.estimated.gps = copy.deepcopy(gps_reading)
        self.estimated.adc = copy.deepcopy(adc_reading)

    def get_estimated_state(self, use_filtered=True) -> EstimatedState:
        if use_filtered:
            return self.estimated
        else:
            return self.raw
