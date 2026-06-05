"""
Flight Data Recorder (FDR).
Captures simulation state, controls, and sensor data to CSV format.
Implements Chunked Writing to prevent Out-Of-Memory (OOM) during long simulations.

飞行数据记录器 (FDR)。
负责将仿真过程中的状态、控制量、传感器数据保存为 CSV 格式。
实现分块写入机制，防止长航时仿真导致内存溢出 (OOM)。
"""
import csv
import numpy as np
import os

class DataLogger:
    def __init__(self, filename="flight_data.csv", chunk_size=1000):
        """
        Args:
            filename: Output CSV file path.
            chunk_size: Number of rows to keep in memory before writing to disk.
                        在写入磁盘前保留在内存中的行数。
        """
        self.filename = filename
        self.chunk_size = chunk_size
        self.data_buffer = []

        # CSV Header Definition / CSV 表头定义
        self.headers = [
            "Time", "PosX", "PosY", "Alt",
            "VelN", "VelE", "VelD",
            "Roll", "Pitch", "Yaw",
            "Qw", "Qx", "Qy", "Qz",
            "TAS", "Alpha", "Beta",
            "Elevator", "Aileron", "Rudder", "Throttle",
            "AccZ", "Lat", "Lon", "GPS_Alt", "GPS_Spd"
        ]

        # Initialize file with headers
        # 初始化文件并写入表头
        self._init_file()

    def _init_file(self):
        """Creates a new file and writes the header row."""
        try:
            with open(self.filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.headers)
        except Exception as e:
            print(f"[Logger] Error initializing CSV: {e}")

    def _flush_buffer(self):
        """Appends the current buffer to the CSV file and clears it."""
        if not self.data_buffer:
            return

        try:
            # Open in 'append' mode ('a')
            with open(self.filename, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(self.data_buffer)
            # Clear the buffer after successful write
            self.data_buffer.clear()
        except Exception as e:
            print(f"[Logger] Error flushing data to CSV: {e}")

    def log(self, time, state, controls, air_data, acc_z, gps_data):
        """
        Captures a single frame of simulation data.
        If buffer reaches chunk_size, it automatically flushes to disk.

        记录单帧仿真数据。
        如果缓冲区达到 chunk_size，则自动写入磁盘。
        """
        euler_deg = np.degrees(state.euler_angles)
        alpha_deg = np.degrees(air_data.alpha)
        beta_deg = np.degrees(getattr(air_data, 'beta', 0.0))

        qw, qx, qy, qz = state.q

        row = [
            f"{time:.4f}",
            f"{state.pos[0]:.4f}", f"{state.pos[1]:.4f}", f"{-state.pos[2]:.4f}",
            f"{state.vel[0]:.4f}", f"{state.vel[1]:.4f}", f"{state.vel[2]:.4f}",
            f"{euler_deg[0]:.4f}", f"{euler_deg[1]:.4f}", f"{euler_deg[2]:.4f}",
            f"{qw:.4f}", f"{qx:.4f}", f"{qy:.4f}", f"{qz:.4f}",
            f"{air_data.airspeed_tas:.4f}", f"{alpha_deg:.4f}", f"{beta_deg:.4f}",
            f"{controls.elevator:.4f}", f"{controls.aileron:.4f}", f"{controls.rudder:.4f}", f"{controls.throttle:.4f}",
            f"{acc_z:.4f}",
            f"{gps_data.latitude:.6f}", f"{gps_data.longitude:.6f}",
            f"{gps_data.altitude:.2f}", f"{gps_data.ground_speed:.2f}"
        ]

        self.data_buffer.append(row)

        # Check if we need to flush
        if len(self.data_buffer) >= self.chunk_size:
            self._flush_buffer()

    def close(self):
        """
        Flushes any remaining data in the buffer. Should be called at the end of simulation.
        将缓冲区中剩余的数据写入磁盘。应在仿真结束时调用。
        """
        self._flush_buffer()
        print(f"[Logger] Final data saved to {self.filename}")