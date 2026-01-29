"""
Example Script: Su-27 Cobra Maneuver (Real Physics / Pilot Skill Mode).
Demonstrates high-alpha aerodynamics using ONLY control inputs.
NO config modifications allowed.

Strategy:
1. Aggressive Entry: Full back stick for longer duration to build rotational inertia.
2. Idle Throttle: Prevents climbing, forces deep stall.
3. Late Braking: Don't push forward until we hit 90+ degrees.
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
from modules.sensors.gps import GPSReading

def main():
    print("=== Example: Su-27 Cobra Maneuver (Pilot Skill Mode) ===")

    # 1. Load Su-27 Config (REAL DATA)
    try:
        ac_cfg = Loader.load_yaml("configs/su27.yaml")
        print("Loaded Su-27 Flanker configuration.")
    except FileNotFoundError:
        print("Error: configs/su27.yaml not found.")
        return

    # 2. Init Physics
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
        stall_width=ap.get('stall_width', 0.15)
    )
    aero = Aerodynamics(aero_params)

    imu = IMU()
    adc = AirDataComputer()
    logger = DataLogger()

    # 3. Initial State
    # Su-27 Cobra Entry: ~450 km/h (125 m/s)
    init_pos = np.array([0, 0, -3000])
    init_vel = np.array([125, 0, 0])
    init_att = np.array([1, 0, 0, 0])
    aircraft = AircraftState(pos=init_pos, vel=init_vel, att=init_att)

    controls = ControlInputs()
    dt = 0.01 # High fidelity step
    t_max = 15.0
    num_steps = int(t_max / dt)
    temp_state = AircraftState()

    print(f"Simulating {t_max}s...")

    for step in range(num_steps):
        t = step * dt

        # Sensor Data
        pitch_rad = imu.get_data().euler[1]
        pitch_deg = np.degrees(pitch_rad)
        pitch_rate = aircraft.rates[1] # q (rad/s)
        alpha_deg = np.degrees(adc.get_reading().alpha)

        # --- COBRA CONTROL LOGIC (The "Ace Pilot") ---

        # Phase 1: Entry (Trim)
        if t < 1.0:
            controls.elevator = 0.0
            controls.throttle = 0.6

        # Phase 2: THE HAMMER PULL (Initiate)
        # Pull HARD and hold it longer to overcome stability.
        # Cut throttle to Idle to stop climbing.
        elif t < 3.5:
            controls.elevator = -1.0 # Full Back Stick
            controls.throttle = 0.0  # Idle

        # Phase 3: THE CATCH (Dynamic Braking)
        # Wait until we are DEEP into the stall before pushing.
        elif t < 7.0:
            controls.throttle = 0.0 # Keep idle

            # Logic:
            # If Pitch > 110: EMERGENCY PUSH (Don't flip)
            # If Pitch > 90:  Start pushing based on rate
            # If Pitch < 90:  KEEP PULLING! (We need momentum)

            if pitch_deg > 110:
                controls.elevator = 1.0 # Full Push
            elif pitch_deg > 90:
                # We are in the zone. Dampen the rotation.
                # If rotating up fast, push hard.
                controls.elevator = 1.0 * pitch_rate
                controls.elevator = np.clip(controls.elevator, 0.0, 1.0)
            else:
                # Not there yet! Keep pulling to fight stability!
                controls.elevator = -1.0

        # Phase 4: RECOVERY (Power & Stabilize)
        else:
            # Nose is dropping naturally due to stability.
            # Add MAX POWER to fly out of the stall.
            controls.throttle = 1.0

            # SAS to catch the nose at horizon
            kp = 0.15
            kd = 0.8
            sas_cmd = (kp * pitch_rad) + (kd * pitch_rate)
            controls.elevator = np.clip(sas_cmd, -1.0, 1.0)

        # --- Physics Loop ---
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

        # Log
        logger.log(t, aircraft, controls, adc.get_reading(),
                   imu.get_data().specific_force[2], GPSReading())

        if step % int(0.5/dt) == 0:
            tas = adc.get_reading().airspeed_tas
            print(f"T={t:4.1f} | Pitch={pitch_deg:5.1f}° | Alpha={alpha_deg:5.1f}° | TAS={tas:5.1f} | Elev={controls.elevator:.2f}")

    output_file = "cobra_data.csv"
    logger.save_to_csv(output_file)
    print(f"Done. Run 'python plot_results.py --file {output_file} --mode quat' to view.")

if __name__ == "__main__":
    main()