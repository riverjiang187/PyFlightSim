"""
Flight Data Visualization Tool.
Command-line driven plotting utility.
Supports Euler and Quaternion modes.

Usage:
    python plot_results.py --file flight_data.csv --mode euler
    python plot_results.py --file cobra_data.csv --mode quat
"""

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
import argparse
import sys
import os

# Path fix to import tools
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from tools.loader import Loader

def plot_flight_data(filename, mode):
    print(f"Loading {filename} in [{mode.upper()}] mode...")

    try:
        df = Loader.load_csv(filename)
    except FileNotFoundError as e:
        print(e)
        return

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'ggplot')
    fig, axs = plt.subplots(4, 3, figsize=(18, 12))
    fig.suptitle(f'Flight Analysis: {filename} ({mode.upper()})', fontsize=16)

    # --- Row 1: Navigation ---
    axs[0, 0].plot(df['Time'], df['Alt'], 'b'); axs[0, 0].set_title('Altitude (m)')

    if 'Lat' in df.columns:
        axs[0, 1].plot(df['Lon'], df['Lat'], 'purple')
        axs[0, 1].plot(df['Lon'].iloc[0], df['Lat'].iloc[0], 'go', label='Start')
        axs[0, 1].plot(df['Lon'].iloc[-1], df['Lat'].iloc[-1], 'rx', label='End')
        axs[0, 1].set_title('GPS Ground Track'); axs[0, 1].axis('equal')
        axs[0, 1].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].legend()
    else:
        axs[0, 1].plot(df['PosY'], df['PosX'], 'purple'); axs[0, 1].set_title('Local Track')

    axs[0, 2].plot(df['Time'], df['TAS'], 'g'); axs[0, 2].set_title('TAS (m/s)')

    # --- Row 2: Attitude (Switchable) ---
    if mode == 'quat':
        if 'Qw' not in df.columns:
            print("Error: CSV does not contain Quaternion data.")
            return
        axs[1, 0].plot(df['Time'], df['Qw'], 'tab:brown'); axs[1, 0].set_title('Qw')
        axs[1, 1].plot(df['Time'], df['Qx'], 'tab:orange'); axs[1, 1].set_title('Qx')
        axs[1, 2].plot(df['Time'], df['Qy'], 'tab:green', label='Qy')
        axs[1, 2].plot(df['Time'], df['Qz'], 'tab:blue', label='Qz')
        axs[1, 2].set_title('Qy & Qz'); axs[1, 2].legend()
    else:
        # Euler Mode (Default) with Unwrapping
        for col in ['Roll', 'Pitch', 'Yaw']:
            if col in df.columns:
                df[col] = np.degrees(np.unwrap(np.radians(df[col])))

        axs[1, 0].plot(df['Time'], df['Roll'], 'tab:orange'); axs[1, 0].set_title('Roll (deg)')
        axs[1, 1].plot(df['Time'], df['Pitch'], 'r'); axs[1, 1].set_title('Pitch (deg)')
        axs[1, 2].plot(df['Time'], df['Yaw'], 'tab:brown'); axs[1, 2].set_title('Yaw (deg)')

    # --- Row 3: Aerodynamics ---
    for col in ['Alpha', 'Beta']:
        if col in df.columns: df[col] = np.degrees(np.unwrap(np.radians(df[col])))

    axs[2, 0].plot(df['Time'], df['Alpha'], 'm'); axs[2, 0].set_title('Alpha (deg)')
    if 'Beta' in df.columns: axs[2, 1].plot(df['Time'], df['Beta'], 'c'); axs[2, 1].set_title('Beta (deg)')
    axs[2, 2].plot(df['Time'], df['AccZ'], 'k'); axs[2, 2].set_title('Acc Z (m/s^2)')

    # --- Row 4: Controls ---
    axs[3, 0].plot(df['Time'], df['Elevator'], 'k'); axs[3, 0].set_title('Elevator')
    if 'Aileron' in df.columns:
        axs[3, 1].plot(df['Time'], df['Aileron'], 'tab:blue', label='Ail')
        axs[3, 1].plot(df['Time'], df['Rudder'], 'tab:orange', label='Rud', linestyle='--')
        axs[3, 1].legend(); axs[3, 1].set_title('Aileron & Rudder')
    axs[3, 2].plot(df['Time'], df['Throttle'], 'orange'); axs[3, 2].set_title('Throttle')

    plt.tight_layout(); plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Flight Data Plotter")
    parser.add_argument("--file", type=str, default="flight_data.csv", help="CSV file to plot")
    parser.add_argument("--mode", type=str, choices=['euler', 'quat'], default='euler',
                        help="Plotting mode: 'euler' or 'quat'")
    args = parser.parse_args()

    plot_flight_data(args.file, args.mode)