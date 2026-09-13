import pytest
import numpy as np
from modules.sensors.air_data import AirDataComputer
from modules.dynamics.state import AircraftState

def test_pure_headwind():
    """
    纯逆风测试：飞机保持地面速度 60 m/s，施加 10 m/s 机体正前向逆风，
    ADC 读数 TAS 严格等于 70 m/s， alpha = 0, beta = 0。
    """
    adc = AirDataComputer()
    state = AircraftState(vel=np.array([60.0, 0.0, 0.0]))
    # 逆风是迎面吹来的风，机体坐标系下风速向量指向负 X 轴
    wind = np.array([-10.0, 0.0, 0.0])
    
    adc.update(state, wind)
    reading = adc.get_reading()
    
    assert np.isclose(reading.airspeed_tas, 70.0), f"Expected TAS 70.0, got {reading.airspeed_tas}"
    assert np.isclose(reading.alpha, 0.0), f"Expected alpha 0.0, got {reading.alpha}"
    assert np.isclose(reading.beta, 0.0), f"Expected beta 0.0, got {reading.beta}"

def test_pure_crosswind():
    """
    纯侧风测试：施加 5 m/s 右侧风，ADC 读出 beta ≈ arcsin(-5 / sqrt(60^2 + 5^2)) ≈ -4.75°。
    """
    adc = AirDataComputer()
    state = AircraftState(vel=np.array([60.0, 0.0, 0.0]))
    # 右侧风：从右向左吹，即风速在机体坐标系 Y 轴为正
    wind = np.array([0.0, 5.0, 0.0])
    
    adc.update(state, wind)
    reading = adc.get_reading()
    
    expected_beta = np.arcsin(-5.0 / np.sqrt(60**2 + 5**2))
    expected_tas = np.sqrt(60**2 + 5**2)
    
    assert np.isclose(reading.airspeed_tas, expected_tas), f"Expected TAS {expected_tas}, got {reading.airspeed_tas}"
    assert np.isclose(reading.beta, expected_beta), f"Expected beta {expected_beta}, got {reading.beta}"
    assert np.isclose(reading.alpha, 0.0), f"Expected alpha 0.0, got {reading.alpha}"
