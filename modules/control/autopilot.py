"""
Autopilot Core Logic.
Implements Advanced Control Laws:
1. Vertical Speed Mode (Altitude -> Climb Rate -> Pitch).
2. Yaw Damper (Yaw Rate -> Rudder).
3. Turn Coordinator (Beta -> Rudder) [NEW].
4. Gain Scheduling.

自动驾驶仪核心逻辑。
实现高级控制律：
1. 垂直速度模式 (高度 -> 爬升率 -> 俯仰)。
2. 偏航阻尼器 (偏航角速度 -> 方向舵)。
3. 转弯协调器 (侧滑角 -> 方向舵) [新增]。
4. 增益调度。
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

        # --- FIX: Beta Feedback Controller Sign ---
        # 修复：侧滑角反馈控制器的符号
        # Target Beta is 0.0.
        # If Beta > 0 (Nose left), Error = 0 - Beta < 0.
        # We need Rudder > 0 (Yaw Right) to correct.
        # Therefore, Kp must be NEGATIVE.
        # 因此，Kp 必须是负数。
        self.beta_pid = PID(kp=-2.0, ki=-0.1, kd=0.0, out_min=-0.5, out_max=0.5)

        self.k_ari = config.get('turn_coordinator', {}).get('k_ari', 0.2)

        # --- Speed ---
        s = config['speed_hold']
        self.speed_pid = PID(s['kp'], s['ki'], s['kd'], s['out_min'], s['out_max'])

        self.target_alt = 2000.0
        self.target_spd = 60.0
        self.target_hdg = 0.0

        self.yaw_rate_lowpass = 0.0
        self.washout_time_constant = 2.0

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

        # --- Directional Control (Rudder) ---

        # A. Yaw Damper (High-pass filtered yaw rate)
        raw_yaw_rate = imu.angular_rates[2]
        alpha_filter = dt / (self.washout_time_constant + dt)
        self.yaw_rate_lowpass = (1.0 - alpha_filter) * self.yaw_rate_lowpass + alpha_filter * raw_yaw_rate
        washed_yaw_rate = raw_yaw_rate - self.yaw_rate_lowpass
        yaw_damp_cmd = self.yaw_damp_pid.update(0.0, washed_yaw_rate, dt, scale=scaling)

        # B. Turn Coordinator (Beta Feedback)
        # Target Beta is 0.0
        beta_cmd = self.beta_pid.update(0.0, air_data.beta, dt, scale=scaling)

        # C. Aileron-Rudder Interconnect (Feedforward)
        ari_cmd = self.k_ari * roll_cmd

        # Final Rudder Command = Damper + Beta Feedback + ARI
        rud_cmd = np.clip(yaw_damp_cmd + beta_cmd + ari_cmd, -1.0, 1.0)

        # =========================================================
        # 3. Speed Control
        # =========================================================
        thr_cmd = self.speed_pid.update(self.target_spd, air_data.airspeed_tas, dt)

        return pitch_cmd, roll_cmd, rud_cmd, thr_cmd