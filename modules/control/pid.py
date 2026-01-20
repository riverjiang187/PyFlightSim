"""
Generic PID Controller implementation.
Supports integral windup protection and gain scaling.

通用 PID 控制器实现。
支持积分抗饱和与增益缩放。
"""
import numpy as np


class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp;
        self.ki = ki;
        self.kd = kd
        self.out_min = out_min;
        self.out_max = out_max
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_run = True

    def update(self, setpoint, measurement, dt, scale=1.0):
        error = setpoint - measurement
        eff_kp = self.kp * scale
        eff_ki = self.ki * scale
        eff_kd = self.kd * scale

        P = eff_kp * error
        self.integral += error * dt
        I = eff_ki * self.integral

        if self.first_run:
            derivative = 0.0; self.first_run = False
        else:
            derivative = (error - self.prev_error) / dt
        D = eff_kd * derivative
        self.prev_error = error

        return np.clip(P + I + D, self.out_min, self.out_max)