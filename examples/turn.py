"""
Example: Coordinated Turn (Closed-Loop).
Demonstrates Bank-to-Turn logic and Turn Coordinator (ARI).
"""
import sys
import os
import numpy as np

# Path Fix
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from tools.loader import Loader
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
    print("=== Example: Coordinated Turn (Closed-Loop) ===")

    # Load Configs
    ac_cfg = Loader.load_yaml("configs/aircraft.yaml")
    ap_cfg = Loader.load_yaml("configs/autopilot.yaml")

    mp = ac_cfg['mass_props']
    mass_props = MassProperties(mp['mass'], mp['Ixx'], mp['Iyy'], mp['Izz'])
    kinematics = Kinematics(mass_props)

    ap = ac_cfg['aero_params']
    aero = Aerodynamics(AeroParams(**{k: v for k, v in ap.items() if k in AeroParams.__annotations__}))

    imu = IMU()
    adc = AirDataComputer()
    gps = GPS()
    logger = DataLogger(filename="turn_data.csv")
    autopilot = Autopilot(ap_cfg)
    mixer = Mixer(ac_cfg.get('mixer_type', 'standard'))

    # Initial State (Level Flight, Heading North)
    init_pos = np.array([0, 0, -2000])
    init_vel = np.array([60, 0, 0])
    init_quat = np.array([1.0, 0.0, 0.0, 0.0])
    aircraft = AircraftState(pos=init_pos, vel=init_vel, att=init_quat)

    controls = ControlInputs()
    dt = 0.02
    t_max = 40.0
    temp_state = AircraftState()

    # Setup Mission: Turn East (90 deg)
    tgt_alt = 2000.0
    tgt_spd = 60.0
    tgt_hdg = 90.0

    print(f"Commanding Turn to Heading {tgt_hdg}°...")

    for step in range(int(t_max/dt)):
        t = step * dt

        # --- MISSION LOGIC ---
        # At T=2.0s, command the turn
        if t >= 2.0 and t < 2.05: # Just set it once
            autopilot.set_targets(tgt_alt, tgt_spd, tgt_hdg)

        # --- Physics Loop ---
        rho, _, _, _ = Atmosphere.get_properties(-aircraft.pos[2])
        # No wind for this test to clearly see the ARI effect
        wind = np.zeros(3)
        forces, moments = aero.get_forces_and_moments(aircraft, rho, controls, wind)

        imu.update(aircraft, forces, mass_props.mass)
        adc.update(aircraft)
        gps.update(aircraft)

        # --- Control ---
        # Let the Autopilot do ALL the work
        p, r, y, th = autopilot.update(imu.get_data(), adc.get_reading(), dt)
        controls = mixer.mix(p, r, y, th)

        # --- Integrate ---
        def wrapper(y_vec):
            temp_state.from_vector(y_vec)
            r_env, _, _, _ = Atmosphere.get_properties(-temp_state.pos[2])
            f, m = aero.get_forces_and_moments(temp_state, r_env, controls, wind)
            return kinematics.get_state_derivative(temp_state, f, m)

        aircraft.from_vector(Integrator.rk4_step(wrapper, aircraft.to_vector(), dt))

        logger.log(t, aircraft, controls, adc.get_reading(), imu.get_data().specific_force[2], gps.get_reading())

        if step % int(2.0/dt) == 0:
            roll_deg = np.degrees(imu.get_data().euler[0])
            heading = np.degrees(imu.get_data().euler[2])
            print(f"T={t:.1f} | Bank={roll_deg:.1f}° | Hdg={heading:.1f}°")

    output_file = "turn_data.csv"
    logger.close()
    print(f"Done. Saved to {output_file}")

if __name__ == "__main__":
    main()