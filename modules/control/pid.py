"""
Generic PID Controller implementation.
Supports integral windup protection (Dynamic Clamping) and gain scaling.

通用 PID 控制器实现。
支持积分抗饱和 (动态钳位) 与增益缩放。
"""
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max

        self.integral = 0.0
        self.prev_error = 0.0
        self.first_run = True

    def update(self, setpoint, measurement, dt, scale=1.0):
        error = setpoint - measurement

        # Apply gain scheduling scaling
        eff_kp = self.kp * scale
        eff_ki = self.ki * scale
        eff_kd = self.kd * scale

        # 1. Proportional Term
        P = eff_kp * error

        # 2. Derivative Term
        if self.first_run:
            derivative = 0.0
            self.first_run = False
        else:
            derivative = (error - self.prev_error) / dt
        D = eff_kd * derivative
        self.prev_error = error

        # 3. Calculate un-clamped output (P + D + current I)
        # 计算未限幅的输出 (用于判断是否饱和)
        current_I = eff_ki * self.integral
        unclamped_output = P + current_I + D

        # 4. Integral Anti-Windup (Dynamic Clamping)
        # 积分抗饱和 (动态钳位)

        # Check if output is saturated
        # 检查输出是否饱和
        is_saturated = (unclamped_output >= self.out_max) or (unclamped_output <= self.out_min)

        # Check if the error is trying to push the output further into saturation
        # 检查误差是否试图将输出进一步推向饱和方向
        # If output is positive and error is positive (assuming positive Kp), it's pushing harder.
        # We use the sign of (error * Kp) to determine the direction the integral wants to go.
        integral_direction = np.sign(error * eff_kp)
        output_direction = np.sign(unclamped_output)

        is_pushing_harder = (integral_direction == output_direction)

        # Only integrate if NOT (saturated AND pushing harder)
        # 只有在 (未饱和) 或 (已饱和但误差试图将其拉回) 时，才累加积分
        if not (is_saturated and is_pushing_harder):
            self.integral += error * dt

        # Recalculate I with the updated (or frozen) integral
        I = eff_ki * self.integral

        # 5. Final Output Clamping
        final_output = np.clip(P + I + D, self.out_min, self.out_max)

        return final_output