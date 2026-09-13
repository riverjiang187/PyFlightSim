import pytest
import numpy as np
from modules.dynamics.forces import AeroParams, ControlInputs, Aerodynamics
from modules.dynamics.state import AircraftState
from modules.dynamics.kinematics import Kinematics, MassProperties
from modules.dynamics.integrator import Integrator

def test_static_lateral_force_and_moment():
    """
    静态单步测试：给定固定侧滑角 β = +5°，验证 Fy < 0 且大小等于 q_bar * S * C_Y_beta * beta；
    同时验证 Roll < 0（左倾恢复力矩）。
    """
    params = AeroParams(
        S=16.2, b=10.9, c=1.5,
        C_L_0=0.25, C_L_alpha=4.5, C_D_0=0.025, K=0.05,
        C_m_0=0.02, C_m_alpha=-0.5, C_m_q=-10.0,
        C_Y_beta=-0.31, C_l_beta=-0.089
    )
    aero = Aerodynamics(params)
    
    beta = np.radians(5)
    V_tas = 50.0
    u = V_tas * np.cos(beta)
    v = V_tas * np.sin(beta)
    w = 0.0
    
    state = AircraftState(vel=np.array([u, v, w]))
    controls = ControlInputs()
    density = 1.225
    
    forces, moments = aero.get_forces_and_moments(state, density, controls)
    
    Fx, Fy, Fz = forces
    Roll, Pitch, Yaw = moments
    
    q_bar = 0.5 * density * V_tas**2
    expected_Fy = q_bar * params.S * (params.C_Y_beta * beta)
    
    assert Fy < 0.0, "Fy should be negative for positive side slip (right slip creates left force)"
    assert np.isclose(Fy, expected_Fy), f"Expected Fy {expected_Fy}, got {Fy}"
    assert Roll < 0.0, "Roll moment should be negative (left roll for right slip)"

def test_dynamic_dutch_roll():
    """
    动态模态测试：在开环小扰动下激发偏航，观测到衰减的二阶荷兰滚振荡收敛，不再发散。
    """
    params = AeroParams(
        S=16.2, b=10.9, c=1.5,
        C_L_0=0.25, C_L_alpha=4.5, C_D_0=0.025, K=0.05,
        C_m_0=0.02, C_m_alpha=-0.5, C_m_q=-10.0,
        C_Y_beta=-0.31, C_Y_delta_r=0.187,
        C_l_beta=-0.089, C_l_delta_r=0.010,
        C_n_beta=0.1, C_n_r=-0.2, C_l_p=-0.4
    )
    aero = Aerodynamics(params)
    mass = MassProperties(1300.0, 1000.0, 4000.0, 4500.0)
    kinematics = Kinematics(mass)
    
    # 初始状态给一个偏航扰动 r = 0.1 rad/s
    V_tas = 50.0
    state = AircraftState(
        vel=np.array([V_tas, 0.0, 0.0]),
        rates=np.array([0.0, 0.0, 0.1])
    )
    controls = ControlInputs()
    density = 1.225
    
    dt = 0.02
    
    def deriv_func(vec):
        st = AircraftState()
        st.from_vector(vec)
        f, m = aero.get_forces_and_moments(st, density, controls)
        return kinematics.get_state_derivative(st, f, m)
    
    y = state.to_vector()
    beta_history = []
    
    for _ in range(400): # 8 seconds simulation
        y = Integrator.rk4_step(deriv_func, y, dt)
        st = AircraftState()
        st.from_vector(y)
        u, v, w = st.vel
        V = np.linalg.norm(st.vel)
        if V > 0.1:
            beta = np.arcsin(np.clip(v/V, -1, 1))
        else:
            beta = 0.0
        beta_history.append(beta)
        
    peaks = [abs(b) for b in beta_history]
    
    # 简单的阻尼收敛验证：后半段的最大振幅应小于前半段
    max_first_half = max(peaks[:200])
    max_second_half = max(peaks[200:])
    
    assert max_second_half < max_first_half, "Dutch roll should be a damped oscillation"
    assert not np.isnan(y).any(), "Integration diverged (NaN in state)"
