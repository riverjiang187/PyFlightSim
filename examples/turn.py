"""
Example: Coordinated Turn.
Demonstrates Bank-to-Turn logic and lateral stability.
"""
import sys
import os
import numpy as np

# Path Fix
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from tools.loader import Loader # <--- UPDATED
from tools.logger import DataLogger
from modules.dynamics.state import AircraftState
from modules.dynamics.kinematics import Kinematics, MassProperties
from modules.dynamics.integrator import Integrator
from modules.dynamics.forces import Aerodynamics, AeroParams, ControlInputs
from modules.environment.atmosphere import Atmosphere
from modules.sensors.imu import IMU
from modules.sensors.air_data import AirDataComputer
from modules.sensors.gps import GPS
from modules.control.autopilot import Autopilot
from modules.control.mixer import Mixer

def main():
    print("=== Example: Coordinated Turn ===")

    # Load Configs
    ac_cfg = Loader.load_yaml("configs/aircraft.yaml")
    ap_cfg = Loader.load_yaml("configs/autopilot.yaml")

    mp = ac_cfg['mass_props']
    mass_props = MassProperties(mp['mass'], mp['Ixx'], mp['Iyy'], mp['Izz'])
    kinematics = Kinematics(mass_props)

    ap = ac_cfg['aero_params']
    aero_params = AeroParams(
        S=ap['S'], b=ap['b'], c=ap['c'],
        C_L_0=ap['C_L_0'], C_L_alpha=ap['C_L_alpha'],
        C_D_0=ap['C_D_0'], K=ap['K'],
        C_m_0=ap['C_m_0'], C_m_alpha=ap['C_m_alpha'], C_m_q=ap['C_m_q'],
        # Ensure stall parameters are loaded
        alpha_stall=ap.get('alpha_stall', 0.61),
        stall_width=ap.get('stall_width', 0.15),
        max_thrust=ap.get('max_thrust', 5000.0)
    )
    aero = Aerodynamics(aero_params)

    imu = IMU()
    adc = AirDataComputer()
    gps = GPS()
    logger = DataLogger()
    autopilot = Autopilot(ap_cfg)
    mixer = Mixer(ac_cfg.get('mixer_type', 'standard'))

    # Setup: 90 degree turn
    tgt_alt = 2000.0
    tgt_spd = 60.0
    tgt_hdg = 90.0 # Turn East
    autopilot.set_targets(tgt_alt, tgt_spd, tgt_hdg)

    init_pos = np.array([0, 0, -2000])
    init_vel = np.array([60, 0, 0])
    init_quat = np.array([1.0, 0.0, 0.0, 0.0])
    aircraft = AircraftState(pos=init_pos, vel=init_vel, att=init_quat)

    controls = ControlInputs()
    dt = 0.02
    t_max = 40.0
    temp_state = AircraftState()

    print(f"Turning to Heading {tgt_hdg}...")

    for step in range(int(t_max/dt)):
        t = step * dt

        rho, _, _, _ = Atmosphere.get_properties(-aircraft.pos[2])
        forces, moments = aero.get_forces_and_moments(aircraft, rho, controls)

        imu.update(aircraft, forces, mass_props.mass)
        adc.update(aircraft)
        gps.update(aircraft)

        p, r, y, th = autopilot.update(imu.get_data(), adc.get_reading(), dt)
        controls = mixer.mix(p, r, y, th)

        def wrapper(y):
            temp_state.from_vector(y)
            r_env, _, _, _ = Atmosphere.get_properties(-temp_state.pos[2])
            f, m = aero.get_forces_and_moments(temp_state, r_env, controls)
            return kinematics.get_state_derivative(temp_state, f, m)

        aircraft.from_vector(Integrator.rk4_step(wrapper, aircraft.to_vector(), dt))

        logger.log(t, aircraft, controls, adc.get_reading(), imu.get_data().specific_force[2], gps.get_reading())

        if step % int(2.0/dt) == 0:
            roll_deg = np.degrees(imu.get_data().euler[0])
            heading = np.degrees(imu.get_data().euler[2])
            print(f"T={t:.1f} | Bank={roll_deg:.1f}° | Hdg={heading:.1f}°")

    output_file = "turn_data.csv"
    logger.save_to_csv(output_file)
    print(f"Done. Saved to {output_file}")

if __name__ == "__main__":
    main()