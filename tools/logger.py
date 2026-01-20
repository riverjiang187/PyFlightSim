"""
Flight Data Recorder (FDR).
Captures simulation state, controls, and sensor data to CSV format.
Updated to include raw Quaternions for stability analysis.

飞行数据记录器 (FDR)。
负责将仿真过程中的状态、控制量、传感器数据保存为 CSV 格式。
已更新：包含原始四元数数据以供稳定性分析。
"""
import csv
import numpy as np

class DataLogger:
    def __init__(self):
        self.data = []
        # CSV Header Definition / CSV 表头定义
        self.headers = [
            "Time", "PosX", "PosY", "Alt",
            "VelN", "VelE", "VelD",
            "Roll", "Pitch", "Yaw",
            "Qw", "Qx", "Qy", "Qz",      # <--- NEW: Quaternions / 新增：四元数
            "TAS", "Alpha", "Beta",
            "Elevator", "Aileron", "Rudder", "Throttle",
            "AccZ", "Lat", "Lon", "GPS_Alt", "GPS_Spd"
        ]

    def log(self, time, state, controls, air_data, acc_z, gps_data):
        """
        Captures a single frame of simulation data.
        记录单帧仿真数据。
        """
        euler_deg = np.degrees(state.euler_angles)
        alpha_deg = np.degrees(air_data.alpha)
        beta_deg = np.degrees(getattr(air_data, 'beta', 0.0))

        # Extract Quaternion (Scalar first: w, x, y, z)
        # 提取四元数 (实部在前)
        qw, qx, qy, qz = state.q

        row = [
            f"{time:.4f}",
            f"{state.pos[0]:.4f}", f"{state.pos[1]:.4f}", f"{-state.pos[2]:.4f}",
            f"{state.vel[0]:.4f}", f"{state.vel[1]:.4f}", f"{state.vel[2]:.4f}",
            f"{euler_deg[0]:.4f}", f"{euler_deg[1]:.4f}", f"{euler_deg[2]:.4f}",
            f"{qw:.4f}", f"{qx:.4f}", f"{qy:.4f}", f"{qz:.4f}", # <--- Log Quats
            f"{air_data.airspeed_tas:.4f}", f"{alpha_deg:.4f}", f"{beta_deg:.4f}",
            f"{controls.elevator:.4f}", f"{controls.aileron:.4f}", f"{controls.rudder:.4f}", f"{controls.throttle:.4f}",
            f"{acc_z:.4f}",
            f"{gps_data.latitude:.6f}", f"{gps_data.longitude:.6f}",
            f"{gps_data.altitude:.2f}", f"{gps_data.ground_speed:.2f}"
        ]
        self.data.append(row)

    def save_to_csv(self, filename="flight_data.csv"):
        """
        Writes buffer to disk.
        将缓冲区数据写入磁盘。
        """
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.headers)
                writer.writerows(self.data)
            print(f"[Logger] Data saved to {filename}")
        except Exception as e:
            print(f"[Logger] Error saving CSV: {e}")