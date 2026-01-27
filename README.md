# PyFlightSim: High-Fidelity 6-DOF Flight Dynamics Engine

**PyFlightSim** is a modular, data-driven, and physics-accurate 6-DOF flight simulation framework written in pure Python. 

It is designed as a **"White-Box" engineering tool** for research, control algorithm validation (PID/LQR/RL), and flight dynamics education. Unlike game engines, it provides full access to the underlying physics equations and control loops.

![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Stable-brightgreen)

## 🌟 Key Features

*   **Industrial-Grade Physics Core**:
    *   Full 6-DOF Equations of Motion (Newton-Euler).
    *   **Quaternion-based** kinematics (Eliminates Gimbal Lock).
    *   **RK4 Integration** for high-precision physics stepping.
*   **Advanced Control System**:
    *   Cascaded PID Autopilot (Altitude / Heading / Speed).
    *   **Gain Scheduling** based on dynamic pressure.
    *   **Yaw Damper** & Turn Coordination logic.
*   **Realistic Environment**:
    *   ISA Standard Atmosphere model.
    *   **Dryden Turbulence Model** for robust control testing.
*   **Data-Driven Architecture**:
    *   Fully configurable via YAML (Aircraft, Autopilot, Simulation).
    *   Decoupled design: Physics, Sensors, and Control are isolated.

## 📂 Project Structure

```text
PyFlightSim/
├── run_simulation.py    # [Entry] Simulation Entry Point
├── plot_results.py      # [Entry] Visualization Tool
├── configs/             # [Data] Configuration Files
│   ├── aircraft.yaml    # Physics parameters
│   ├── autopilot.yaml   # PID gains
│   └── simulation.yaml  # Scenario settings
├── modules/             # [Core] The Physics Engine
│   ├── dynamics/        # 6-DOF EOM & Kinematics
│   ├── control/         # PID & Autopilot Logic
│   ├── sensors/         # IMU, GPS, AirData
│   └── environment/     # Atmosphere & Turbulence
├── tools/               # [Utils] Loader & Logger
└── examples/            # [Demo] Pre-configured scenarios
```

## 🚀 Quick Start

### 1. Installation
Clone the repository and install dependencies:
```bash
git clone https://github.com/riverjiang187/PyFlightSim.git
cd PyFlightSim
pip install -r requirements.txt
```

### 2. Run Simulation
Execute the standard cruise mission (Cessna 172):
```bash
python run_simulation.py
```
*The simulation will run for 60s, simulating a climb to 2000m in turbulence.*

### 3. Visualize Results
Plot the flight data (Altitude, Attitude, Controls, etc.):
```bash
# Standard Mode (Euler Angles)
python plot_results.py --file flight_data.csv --mode euler

# Hardcore Mode (Quaternions)
python plot_results.py --file flight_data.csv --mode quat
```

## 🎮 Advanced Usage

### F-16 Cobra Maneuver
Simulate a post-stall maneuver using the F-16 configuration. This demonstrates the robustness of the quaternion kernel at 90° pitch.
```bash
python examples/run_cobra.py
python plot_results.py --file cobra_data.csv --mode quat
```

### Coordinated Turn
Test lateral stability and bank-to-turn logic:
```bash
python examples/run_turn.py
python plot_results.py --file turn_data.csv --mode euler
```

## 🛠️ Configuration

You can modify flight behavior by editing files in `configs/`:
*   `simulation.yaml`: Change time step, duration, turbulence intensity.
*   `autopilot.yaml`: Tune PID gains and limits.
*   `aircraft.yaml`: Modify mass, inertia, and aerodynamic coefficients.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
