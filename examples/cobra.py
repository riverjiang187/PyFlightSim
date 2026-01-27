"""
Example Script: F-16 Cobra Maneuver.
Demonstrates high-alpha aerodynamics and gimbal lock prevention.
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
from modules.sensors.gps import GPSReading

def main():
    print("=== Example: F-16 Cobra Maneuver ===")

    # Load F-16 Config using Loader
    try:
        ac_cfg = Loader.load_yaml("configs/f16.yaml")
        print("Loaded F-16 configuration.")
    except FileNotFoundError:
        print("Warning: configs/f16.yaml not found. Using standard aircraft.yaml.")
        ac_cfg = Loader.load_yaml("configs/aircraft.yaml")

    # ... (Rest of the physics init is same as before) ...
    mp = ac_cfg['mass_props']
    mass_props = MassProperties(mp['mass'], mp['Ixx'], mp['Iyy'], mp['Izz'])
    kinematics = Kinematics(mass_props)

    ap = ac_cfg['aero_params']
    aero_params = AeroParams(
        S=ap['S'], b=ap['b'], c=ap['c'],
        C_L_0=ap['C_L_0'], C_L_alpha=ap['C_L_alpha'],
        C_D_0=ap['C_D_0'], K=ap['K'],
        C_m_0=ap['C_m_0'], C_m_alpha=ap['C_m_alpha'], C_m_q=ap['C_m_q'],
        alpha_stall=0.436, stall_width=0.08
    )
    aero = Aerodynamics(aero_params)

    imu = IMU()
    adc = AirDataComputer()
    logger = DataLogger()

    # Initial State
    init_pos = np.array([0, 0, -3000])
    init_vel = np.array([150, 0, 0])
    init_att = np.array([1, 0, 0, 0])
    aircraft = AircraftState(pos=init_pos, vel=init_vel, att=init_att)

    controls = ControlInputs()
    dt = 0.01
    t_max = 12.0
    num_steps = int(t_max / dt)
    temp_state = AircraftState()

    print(f"Simulating {t_max}s...")

    for step in range(num_steps):
        t = step * dt

        pitch_rad = imu.get_data().euler[1]
        pitch_deg = np.degrees(pitch_rad)
        pitch_rate = aircraft.rates[1]

        # Maneuver Logic
        if t < 2.0:
            controls.elevator = 0.0
            controls.throttle = 0.6
        elif t < 2.3:
            controls.elevator = -1.0
            controls.throttle = 0.4
        elif t < 4.5:
            if pitch_deg > 85:
                controls.elevator = 0.5 + (2.0 * pitch_rate)
                controls.elevator = np.clip(controls.elevator, 0.0, 1.0)
            else:
                controls.elevator = 0.0
            controls.throttle = 1.0
        else:
            kp = 0.1; kd = 0.8
            sas_cmd = (kp * pitch_rad) + (kd * pitch_rate)
            controls.elevator = np.clip(sas_cmd, -1.0, 1.0)
            controls.throttle = 0.8

        # Physics
        rho, _, _, _ = Atmosphere.get_properties(-aircraft.pos[2])
        forces, moments = aero.get_forces_and_moments(aircraft, rho, controls)

        imu.update(aircraft, forces, mass_props.mass)
        adc.update(aircraft)

        def physics_wrapper(y):
            temp_state.from_vector(y)
            r, _, _, _ = Atmosphere.get_properties(-temp_state.pos[2])
            f, m = aero.get_forces_and_moments(temp_state, r, controls)
            return kinematics.get_state_derivative(temp_state, f, m)

        next_vec = Integrator.rk4_step(physics_wrapper, aircraft.to_vector(), dt)
        aircraft.from_vector(next_vec)

        logger.log(t, aircraft, controls, adc.get_reading(),
                   imu.get_data().specific_force[2], GPSReading())

        if step % int(0.5/dt) == 0:
            alpha_deg = np.degrees(adc.get_reading().alpha)
            tas = adc.get_reading().airspeed_tas
            print(f"T={t:4.1f} | Pitch={pitch_deg:5.1f}° | Alpha={alpha_deg:5.1f}° | TAS={tas:5.1f}")

    output_file = "cobra_data.csv"
    logger.save_to_csv(output_file)
    print(f"Done. Run 'python plot_results.py --file {output_file} --mode quat' to view.")

if __name__ == "__main__":
    main()