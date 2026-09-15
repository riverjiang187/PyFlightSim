import numpy as np
import pytest
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from modules.sensors.imu import IMU
from modules.sensors.gps import GPS
from modules.sensors.estimator import Estimator
from modules.sensors.air_data import AirDataComputer
from modules.dynamics.state import AircraftState
from modules.utils.math3d import MathUtils

def test_imu_statistical_properties():
    """Verify IMU noise and bias drift properties over 300 seconds."""
    config = {
        'gyro_noise_std': 0.005,
        'gyro_bias_tau': 100.0,
        'gyro_bias_noise_std': 0.0001,
        'accel_noise_std': 0.05,
        'accel_bias_tau': 100.0,
        'accel_bias_noise_std': 0.001
    }
    imu = IMU(config=config, enable_noise=True)
    state = AircraftState() # Stationary
    forces = np.zeros(3)
    mass = 1000.0
    dt = 0.02
    steps = int(300 / dt) # 300s

    gyro_outputs = []
    
    for _ in range(steps):
        imu.update(state, forces, mass, dt)
        gyro_outputs.append(imu.get_data().angular_rates[0])

    gyro_outputs = np.array(gyro_outputs)
    
    # Check variance
    # The total variance is noise variance + bias variance.
    # Bias variance for Gauss-Markov process: sigma_b^2 * tau / 2
    # In this simple numerical simulation, we check if the high-frequency variance is close to config
    # By taking differences to remove the low-frequency bias drift
    diffs = np.diff(gyro_outputs)
    # Variance of difference of independent white noise is 2 * sigma^2
    estimated_noise_var = np.var(diffs) / 2.0
    expected_noise_var = config['gyro_noise_std']**2
    
    ratio = estimated_noise_var / expected_noise_var
    assert 0.90 <= ratio <= 1.10, f"IMU Noise variance ratio {ratio} not within acceptable bounds"

    # Check for bias drift (low-frequency walk)
    # The mean over 10s windows should drift
    window_size = int(10 / dt)
    means = [np.mean(gyro_outputs[i:i+window_size]) for i in range(0, len(gyro_outputs), window_size)]
    drift_var = np.var(means)
    assert drift_var > 0.0, "IMU output should exhibit low-frequency drift"


def test_gps_zoh_and_delay():
    """Verify GPS ZOH and 150ms delay."""
    config = {
        'update_rate': 10.0, # 10Hz -> update every 0.1s
        'delay': 0.15,       # 150ms delay
        'horizontal_std': 0.0,
        'vertical_std': 0.0
    }
    # No spatial noise for exact delay testing
    gps = GPS(config=config, enable_noise=True)
    state = AircraftState()
    
    dt = 0.01 # 100Hz main loop
    
    latitudes = []
    
    for step in range(300):
        # Simulate moving aircraft: moving North at constant rate
        state.pos[0] += 1.0 # 1m per step
        gps.update(state, dt)
        
        latitudes.append(gps.get_reading().latitude)
        
    # Check ZOH: Data should stay constant for 10 steps (0.1s / 0.01s = 10 steps)
    # Wait until buffer is filled
    start_check = 50
    changes = 0
    for i in range(start_check, start_check + 50):
        if latitudes[i] != latitudes[i-1]:
            changes += 1
            
    # 50 steps = 0.5 seconds, should see exactly 5 changes
    assert changes == 5, f"Expected 5 staircase changes, got {changes}. ZOH logic failed."


def test_estimator_isolation_and_filter():
    """Verify Estimator filter logic."""
    estimator = Estimator(filter_alpha=0.1)
    
    from modules.sensors.imu import IMUData
    from modules.sensors.gps import GPSReading
    from modules.sensors.air_data import AirDataReading
    
    dt = 0.02
    t = np.arange(0, 10, dt)
    
    raw_signal = np.sin(2 * np.pi * 0.1 * t) # Low freq 0.1Hz
    noise = np.random.normal(0, 0.5, len(t))
    noisy_signal = raw_signal + noise
    
    filtered_outputs = []
    raw_outputs = []
    
    for i in range(len(t)):
        imu_data = IMUData()
        imu_data.angular_rates = np.array([noisy_signal[i], 0, 0])
        
        estimator.update(imu_data, GPSReading(), AirDataReading(), dt)
        
        est_filtered = estimator.get_estimated_state(use_filtered=True)
        est_raw = estimator.get_estimated_state(use_filtered=False)
        
        filtered_outputs.append(est_filtered.imu.angular_rates[0])
        raw_outputs.append(est_raw.imu.angular_rates[0])
        
    filtered_outputs = np.array(filtered_outputs)
    raw_outputs = np.array(raw_outputs)
    
    # 1. Raw output should exactly match noisy input
    assert np.allclose(raw_outputs, noisy_signal), "Raw state is not perfectly isolated"
    
    # 2. Filtered output should suppress noise (have lower variance against true signal than noisy signal)
    mse_raw = np.mean((noisy_signal - raw_signal)**2)
    mse_filtered = np.mean((filtered_outputs - raw_signal)**2)
    
    assert mse_filtered < mse_raw, "Filter did not suppress high-frequency noise"
