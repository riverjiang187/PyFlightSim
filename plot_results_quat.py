"""
Data visualization tool (Quaternion Mode).
Reads 'flight_data.csv' and plots Raw Quaternions (Qw, Qx, Qy, Qz).
Best for verifying physics stability and debugging Gimbal Lock issues.

数据可视化工具 (四元数模式)。
读取 flight_data.csv 并绘制原始四元数 (Qw, Qx, Qy, Qz)。
最适合验证物理稳定性及调试万向节死锁问题。
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
    fig.suptitle(f'Hardcore Analysis (Quaternions): {filename}', fontsize=16)

    # Row 1: Navigation
    axs[0, 0].plot(df['Time'], df['Alt'], 'b');
    axs[0, 0].set_title('Altitude (m)')

    if 'Lat' in df.columns:
        axs[0, 1].plot(df['Lon'], df['Lat'], 'purple')
        axs[0, 1].plot(df['Lon'].iloc[0], df['Lat'].iloc[0], 'go')
        axs[0, 1].plot(df['Lon'].iloc[-1], df['Lat'].iloc[-1], 'rx')
        axs[0, 1].set_title('GPS Ground Track');
        axs[0, 1].axis('equal')
        axs[0, 1].xaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
        axs[0, 1].yaxis.set_major_formatter(ticker.FormatStrFormatter('%.4f'))
    else:
        axs[0, 1].plot(df['PosY'], df['PosX'], 'purple');
        axs[0, 1].set_title('Local Track')

    axs[0, 2].plot(df['Time'], df['TAS'], 'g');
    axs[0, 2].set_title('TAS (m/s)')

    # --- Row 2: Quaternions (The Truth) ---
    if 'Qw' in df.columns:
        # Plot Scalar Part (w)
        axs[1, 0].plot(df['Time'], df['Qw'], 'tab:brown', linewidth=2)
        axs[1, 0].set_title('Quaternion Scalar (Qw)')
        axs[1, 0].set_ylim(-1.1, 1.1)

        # Plot Vector X (x)
        axs[1, 1].plot(df['Time'], df['Qx'], 'tab:orange', linewidth=2)
        axs[1, 1].set_title('Quaternion X (Qx)')
        axs[1, 1].set_ylim(-1.1, 1.1)

        # Plot Vector Y & Z (y, z)
        axs[1, 2].plot(df['Time'], df['Qy'], 'tab:green', label='Qy')
        axs[1, 2].plot(df['Time'], df['Qz'], 'tab:blue', label='Qz')
        axs[1, 2].set_title('Quaternion Y & Z')
        axs[1, 2].set_ylim(-1.1, 1.1)
        axs[1, 2].legend()
    else:
        axs[1, 0].text(0.5, 0.5, "No Quaternion Data", ha='center')

    # Row 3: Aerodynamics
    axs[2, 0].plot(df['Time'], df['Alpha'], 'm');
    axs[2, 0].set_title('Alpha (deg)')
    if 'Beta' in df.columns: axs[2, 1].plot(df['Time'], df['Beta'], 'c'); axs[2, 1].set_title('Beta (deg)')
    axs[2, 2].plot(df['Time'], df['AccZ'], 'k');
    axs[2, 2].set_title('Acc Z (m/s^2)')

    # Row 4: Controls
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