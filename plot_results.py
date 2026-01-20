"""
Data visualization tool.
Reads 'flight_data.csv' and generates a dashboard with 12 subplots.
Covers trajectory, attitude, speed, aerodynamics, and controls.

数据可视化工具。
读取 flight_data.csv 并生成包含 12 个子图的飞行分析仪表板。
涵盖：轨迹、姿态、速度、气动角、控制量等。
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot_flight_data(filename="flight_data.csv"):
    print(f"Reading {filename}...")
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print("File not found."); return

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'ggplot')
    fig, axs = plt.subplots(4, 3, figsize=(18, 12))
    fig.suptitle(f'Full Flight Analysis: {filename}', fontsize=16)

    # Row 1: Navigation & Performance / 导航与性能
    axs[0, 0].plot(df['Time'], df['Alt'], 'b');
    axs[0, 0].set_title('Altitude (m)')

    if 'Lat' in df.columns:
        axs[0, 1].plot(df['Lon'], df['Lat'], 'purple')
        axs[0, 1].plot(df['Lon'].iloc[0], df['Lat'].iloc[0], 'go', label='Start')
        axs[0, 1].plot(df['Lon'].iloc[-1], df['Lat'].iloc[-1], 'rx', label='End')
        axs[0, 1].set_title('GPS Ground Track');
        axs[0, 1].axis('equal')
        axs[0, 1].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].legend()
    else:
        axs[0, 1].plot(df['PosY'], df['PosX'], 'purple');
        axs[0, 1].set_title('Local Track')

    axs[0, 2].plot(df['Time'], df['TAS'], 'g');
    axs[0, 2].set_title('TAS (m/s)')

    # Row 2: Attitude / 姿态
    axs[1, 0].plot(df['Time'], df['Roll'], 'tab:orange');
    axs[1, 0].set_title('Roll (deg)')
    axs[1, 1].plot(df['Time'], df['Pitch'], 'r');
    axs[1, 1].set_title('Pitch (deg)')
    axs[1, 2].plot(df['Time'], df['Yaw'], 'tab:brown');
    axs[1, 2].set_title('Yaw (deg)')

    # Row 3: Aerodynamics & G-Load / 气动与过载
    axs[2, 0].plot(df['Time'], df['Alpha'], 'm');
    axs[2, 0].set_title('Alpha (deg)')
    if 'Beta' in df.columns: axs[2, 1].plot(df['Time'], df['Beta'], 'c'); axs[2, 1].set_title('Beta (deg)')
    axs[2, 2].plot(df['Time'], df['AccZ'], 'k');
    axs[2, 2].set_title('Acc Z (m/s^2)')

    # Row 4: Controls / 控制量
    axs[3, 0].plot(df['Time'], df['Elevator'], 'k');
    axs[3, 0].set_title('Elevator')
    if 'Aileron' in df.columns:
        axs[3, 1].plot(df['Time'], df['Aileron'], 'tab:blue', label='Ail')
        axs[3, 1].plot(df['Time'], df['Rudder'], 'tab:orange', label='Rud', linestyle='--')
        axs[3, 1].legend();
        axs[3, 1].set_title('Aileron & Rudder')
    axs[3, 2].plot(df['Time'], df['Throttle'], 'orange');
    axs[3, 2].set_title('Throttle')

    plt.tight_layout();
    plt.show()


if __name__ == "__main__":
    plot_flight_data()