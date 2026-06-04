"""
Autopilot Core Logic.
Implements 3-Axis Cascaded PID Control using Quaternion-based Error.
Includes Vector-based Heading calculation and Yaw Damper with Washout Filter.

自动驾驶仪核心逻辑。
使用基于四元数误差的三轴级联 PID 控制。
包含基于矢量的航向计算以及带冲刷滤波器的偏航阻尼器。
"""
import numpy as np
from modules.control.pid import PID
from modules.utils.math3d import MathUtils

class Autopilot:
    def __init__(self, config):
        self.design_speed = config.get('design_speed', 60.0)
        self.limits = config.get('limits', {
            'max_climb_rate': 5.0, 'max_sink_rate': 5.0,
            'max_bank': 0.52, 'max_pitch': 0.35
        })

        # --- Longitudinal Loops ---
        a = config['altitude_hold']
        self.alt_pid = PID(a['kp'], a['ki'], a['kd'], a['out_min'], a['out_max'])

        c = config.get('climb_rate_hold', {'kp':0.1, 'ki':0, 'kd':0, 'out_min':-0.3, 'out_max':0.3})
        self.vs_pid = PID(c['kp'], c['ki'], c['kd'], c['out_min'], c['out_max'])

        p = config['pitch_hold']
        self.pitch_pid = PID(p['kp'], p['ki'], p['kd'], p['out_min'], p['out_max'])

        # --- Lateral Loops ---
        h = config['heading_hold']
        self.hdg_pid = PID(h['kp'], h['ki'], h['kd'], h['out_min'], h['out_max'])

        r = config['roll_hold']
        self.roll_pid = PID(r['kp'], r['ki'], r['kd'], r['out_min'], r['out_max'])

        y = config.get('yaw_damper', {'kp':0, 'ki':0, 'kd':0, 'out_min':0, 'out_max':0})
        self.yaw_damp_pid = PID(y['kp'], y['ki'], y['kd'], y['out_min'], y['out_max'])

        # --- Speed ---
        s = config['speed_hold']
        self.speed_pid = PID(s['kp'], s['ki'], s['kd'], s['out_min'], s['out_max'])

        self.target_alt = 2000.0
        self.target_spd = 60.0
        self.target_hdg = 0.0

        # --- FIX: Washout Filter State ---
        # 修复：冲刷滤波器状态变量
        self.yaw_rate_lowpass = 0.0
        self.washout_time_constant = 2.0 # Seconds (Typical value for aircraft)

    def set_targets(self, alt, speed, heading_deg=0.0):
        self.target_alt = alt
        self.target_spd = speed
        self.target_hdg = np.radians(heading_deg)

    def _wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _get_vector_heading_error(self, quat, target_heading):
        R_b_n = MathUtils.quat_to_rotation_matrix(quat)
        body_x_ned = R_b_n @ np.array([1.0, 0.0, 0.0])
        north = body_x_ned[0]
        east = body_x_ned[1]
        horizontal_len = np.sqrt(north**2 + east**2)

        if horizontal_len < 0.1:
            return 0.0

        current_heading = np.arctan2(east, north)
        return self._wrap_angle(target_heading - current_heading)

    def update(self, imu, air_data, dt):
        current_speed = max(air_data.airspeed_tas, 10.0)
        scaling = (self.design_speed / current_speed) ** 2
        scaling = np.clip(scaling, 0.2, 5.0)

        # =========================================================
        # 1. Longitudinal Control
        # =========================================================
        target_vs = self.alt_pid.update(self.target_alt, air_data.altitude_baro, dt)
        target_vs = np.clip(target_vs, -self.limits['max_sink_rate'], self.limits['max_climb_rate'])

        tgt_pitch = self.vs_pid.update(target_vs, air_data.climb_rate, dt, scale=scaling)

        q_tgt_pitch = MathUtils.euler_to_quat(imu.euler[0], tgt_pitch, imu.euler[2])
        pitch_err = MathUtils.get_body_frame_error(imu.quat, q_tgt_pitch)[1]

        pitch_cmd = self.pitch_pid.update(pitch_err, 0.0, dt, scale=scaling)

        # =========================================================
        # 2. Lateral Control
        # =========================================================
        hdg_error = self._get_vector_heading_error(imu.quat, self.target_hdg)
        tgt_roll = self.hdg_pid.update(hdg_error, 0.0, dt)
        tgt_roll = np.clip(tgt_roll, -self.limits['max_bank'], self.limits['max_bank'])

        q_tgt_roll = MathUtils.euler_to_quat(tgt_roll, imu.euler[1], imu.euler[2])
        roll_err = MathUtils.get_body_frame_error(imu.quat, q_tgt_roll)[0]

        roll_cmd = self.roll_pid.update(roll_err, 0.0, dt, scale=scaling)

        # --- FIX: Yaw Damper with Washout Filter ---
        # 修复：带冲刷滤波器的偏航阻尼器
        raw_yaw_rate = imu.angular_rates[2]

        # 1. Low-pass filter the yaw rate to find the "steady state" turn rate
        #    低通滤波提取稳态转弯角速度
        alpha_filter = dt / (self.washout_time_constant + dt)
        self.yaw_rate_lowpass = (1.0 - alpha_filter) * self.yaw_rate_lowpass + alpha_filter * raw_yaw_rate

        # 2. High-pass filter (Washout) = Raw - LowPass
        #    高通滤波 (冲刷) = 原始值 - 低通值
        # This isolates the high-frequency gusts/oscillations
        # 这分离出了高频阵风/震荡
        washed_yaw_rate = raw_yaw_rate - self.yaw_rate_lowpass

        # 3. Feed the washed signal to the PID
        #    将冲刷后的信号喂给 PID
        rud_cmd = self.yaw_damp_pid.update(0.0, washed_yaw_rate, dt, scale=scaling)

        # =========================================================
        # 3. Speed Control
        # =========================================================
        thr_cmd = self.speed_pid.update(self.target_spd, air_data.airspeed_tas, dt)

        return pitch_cmd, roll_cmd, rud_cmd, thr_cmd