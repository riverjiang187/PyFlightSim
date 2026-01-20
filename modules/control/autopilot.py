"""
Autopilot Core Logic.
Implements Advanced Control Laws:
1. Vertical Speed Mode (Altitude -> Climb Rate -> Pitch).
2. Yaw Damper (Yaw Rate -> Rudder).
3. Gain Scheduling.

自动驾驶仪核心逻辑。
实现高级控制律：
1. 垂直速度模式 (高度 -> 爬升率 -> 俯仰)。
2. 偏航阻尼器 (偏航角速度 -> 方向舵)。
3. 增益调度。
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

        # --- Longitudinal Loops / 纵向回路 ---
        # 1. Alt -> Climb Rate
        a = config['altitude_hold']
        self.alt_pid = PID(a['kp'], a['ki'], a['kd'], a['out_min'], a['out_max'])

        # 2. Climb Rate -> Pitch [NEW]
        c = config.get('climb_rate_hold', {'kp':0.1, 'ki':0, 'kd':0, 'out_min':-0.3, 'out_max':0.3})
        self.vs_pid = PID(c['kp'], c['ki'], c['kd'], c['out_min'], c['out_max'])

        # 3. Pitch -> Elevator
        p = config['pitch_hold']
        self.pitch_pid = PID(p['kp'], p['ki'], p['kd'], p['out_min'], p['out_max'])

        # --- Lateral Loops / 横侧向回路 ---
        h = config['heading_hold']
        self.hdg_pid = PID(h['kp'], h['ki'], h['kd'], h['out_min'], h['out_max'])

        r = config['roll_hold']
        self.roll_pid = PID(r['kp'], r['ki'], r['kd'], r['out_min'], r['out_max'])

        # Yaw Damper [NEW]
        y = config.get('yaw_damper', {'kp':0, 'ki':0, 'kd':0, 'out_min':0, 'out_max':0})
        self.yaw_damp_pid = PID(y['kp'], y['ki'], y['kd'], y['out_min'], y['out_max'])

        # --- Speed / 速度 ---
        s = config['speed_hold']
        self.speed_pid = PID(s['kp'], s['ki'], s['kd'], s['out_min'], s['out_max'])

        self.target_alt = 2000.0
        self.target_spd = 60.0
        self.target_hdg = 0.0

    def set_targets(self, alt, speed, heading_deg=0.0):
        self.target_alt = alt
        self.target_spd = speed
        self.target_hdg = np.radians(heading_deg)

    def _wrap_angle(self, angle):
        """
        Keep angle between -pi and pi.
        保持角度在 -pi 到 pi 之间。
        """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def update(self, imu, air_data, dt):
        # Gain Scheduling / 增益调度
        current_speed = max(air_data.airspeed_tas, 10.0)
        scaling = (self.design_speed / current_speed) ** 2
        scaling = np.clip(scaling, 0.2, 5.0)

        # =========================================================
        # 1. Longitudinal Control (Vertical Speed Mode)
        #    纵向控制 (垂直速度模式)
        # =========================================================

        # Step A: Altitude -> Target Climb Rate
        # Limit max climb/sink rate to prevent G-load spikes
        # 限制最大爬升/下降率，防止过载尖峰
        target_vs = self.alt_pid.update(self.target_alt, air_data.altitude_baro, dt)
        target_vs = np.clip(target_vs, -self.limits['max_sink_rate'], self.limits['max_climb_rate'])

        # Step B: Climb Rate -> Target Pitch
        tgt_pitch = self.vs_pid.update(target_vs, air_data.climb_rate, dt, scale=scaling)

        # Step C: Pitch -> Elevator (Quaternion based)
        # Construct a "Pitch Only" target quaternion to measure pitch error
        # 构建“仅俯仰”目标四元数以测量俯仰误差
        q_tgt_pitch = MathUtils.euler_to_quat(0.0, tgt_pitch, imu.euler[2])
        att_err = MathUtils.get_body_frame_error(imu.quat, q_tgt_pitch)
        pitch_err = att_err[1]

        pitch_cmd = self.pitch_pid.update(pitch_err, 0.0, dt, scale=scaling)

        # =========================================================
        # 2. Lateral Control (Heading & Yaw Damper)
        #    横侧向控制 (航向保持 & 偏航阻尼)
        # =========================================================

        # Step A: Heading -> Target Roll
        hdg_error = self._wrap_angle(self.target_hdg - imu.euler[2])
        tgt_roll = self.hdg_pid.update(hdg_error, 0.0, dt)
        tgt_roll = np.clip(tgt_roll, -self.limits['max_bank'], self.limits['max_bank'])

        # Step B: Roll -> Aileron
        # Construct "Roll Only" target (using current pitch/yaw)
        # 构建“仅滚转”目标
        q_tgt_roll = MathUtils.euler_to_quat(tgt_roll, imu.euler[1], imu.euler[2])
        att_err_roll = MathUtils.get_body_frame_error(imu.quat, q_tgt_roll)
        roll_err = att_err_roll[0]

        roll_cmd = self.roll_pid.update(roll_err, 0.0, dt, scale=scaling)

        # Step C: Yaw Damper (Yaw Rate -> Rudder)
        # Target Yaw Rate is 0 (stop oscillation)
        # Rudder opposes Yaw Rate (Negative feedback)
        # 目标偏航角速度为 0 (停止震荡)。方向舵抵抗偏航角速度 (负反馈)。
        yaw_rate = imu.angular_rates[2]
        rud_cmd = self.yaw_damp_pid.update(0.0, yaw_rate, dt, scale=scaling)

        # =========================================================
        # 3. Speed Control
        #    速度控制
        # =========================================================
        thr_cmd = self.speed_pid.update(self.target_spd, air_data.airspeed_tas, dt)

        return pitch_cmd, roll_cmd, rud_cmd, thr_cmd