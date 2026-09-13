import pytest
import numpy as np
from modules.dynamics.forces import AeroParams, ControlInputs, Aerodynamics
from modules.dynamics.state import AircraftState

def test_elevator_default_config():
    """
    回归测试：使用默认配置 (0.5, -1.5) 计算升降舵阶跃下的升力与俯仰力矩
    """
    params = AeroParams(
        S=16.2, b=10.9, c=1.5,
        C_L_0=0.25, C_L_alpha=4.5, C_D_0=0.025, K=0.05,
        C_m_0=0.02, C_m_alpha=-0.5, C_m_q=-10.0
    )
    aero = Aerodynamics(params)
    
    # Static flight condition
    V_tas = 50.0
    state = AircraftState(vel=np.array([V_tas, 0.0, 0.0]))
    controls = ControlInputs(elevator=0.2)  # positive elevator step
    density = 1.225
    
    _, moments = aero.get_forces_and_moments(state, density, controls)
    Pitch_default = moments[1]
    
    # Because alpha=0, q=0, the moment should be purely from C_m_0 and elevator
    q_bar = 0.5 * density * V_tas**2
    C_m = params.C_m_0 + params.C_m_delta_e * controls.elevator
    expected_pitch = q_bar * params.S * params.c * C_m
    
    assert np.isclose(Pitch_default, expected_pitch), f"Expected Pitch {expected_pitch}, got {Pitch_default}"

def test_elevator_dynamic_config():
    """
    配置动态生效测试：修改配置 C_m_delta_e 为 -3.0，验证俯仰力矩精确响应变化。
    """
    params = AeroParams(
        S=16.2, b=10.9, c=1.5,
        C_L_0=0.25, C_L_alpha=4.5, C_D_0=0.025, K=0.05,
        C_m_0=0.02, C_m_alpha=-0.5, C_m_q=-10.0,
        C_m_delta_e=-3.0  # Modified from default -1.5
    )
    aero = Aerodynamics(params)
    
    V_tas = 50.0
    state = AircraftState(vel=np.array([V_tas, 0.0, 0.0]))
    controls = ControlInputs(elevator=0.2)
    density = 1.225
    
    _, moments = aero.get_forces_and_moments(state, density, controls)
    Pitch_modified = moments[1]
    
    q_bar = 0.5 * density * V_tas**2
    C_m = params.C_m_0 + (-3.0) * controls.elevator
    expected_pitch = q_bar * params.S * params.c * C_m
    
    assert np.isclose(Pitch_modified, expected_pitch), f"Expected Pitch {expected_pitch}, got {Pitch_modified}"
