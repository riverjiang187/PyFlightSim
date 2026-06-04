"""
Generic PID Controller implementation.
Supports integral windup protection (Dynamic Clamping), gain scaling,
and D-term low-pass filtering to prevent high-frequency chatter.

通用 PID 控制器实现。
支持积分抗饱和 (动态钳位)、增益缩放，
以及 D 项低通滤波以防止高频颤振。
"""
import numpy as np

class PID:
    def __init__(self, kp, ki, kd, out_min, out_max, tau_d=0.05):
        """
        Args:
            tau_d: Time constant for the derivative low-pass filter (seconds).
                   Larger value = more filtering (smoother but more lag).
                   导数低通滤波器的时间常数 (秒)。值越大滤波越强 (更平滑但延迟更大)。
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.tau_d = tau_d

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0 # Store previous filtered derivative
        self.first_run = True

    def update(self, setpoint, measurement, dt, scale=1.0):
        error = setpoint - measurement

        # Apply gain scheduling scaling
        eff_kp = self.kp * scale
        eff_ki = self.ki * scale
        eff_kd = self.kd * scale

        # 1. Proportional Term
        P = eff_kp * error

        # 2. Derivative Term with Low-Pass Filter
        # 带有低通滤波的微分项
        if self.first_run:
            filtered_derivative = 0.0
            self.first_run = False
        else:
            # Raw derivative / 原始微分
            raw_derivative = (error - self.prev_error) / dt

            # First-order low-pass filter / 一阶低通滤波器
            # alpha = dt / (tau + dt)
            alpha = dt / (self.tau_d + dt)
            filtered_derivative = (1.0 - alpha) * self.prev_derivative + alpha * raw_derivative

        D = eff_kd * filtered_derivative

        # Store states for next step
        self.prev_error = error
        self.prev_derivative = filtered_derivative

        # 3. Calculate un-clamped output (P + D + current I)
        current_I = eff_ki * self.integral
        unclamped_output = P + current_I + D

        # 4. Integral Anti-Windup (Dynamic Clamping)
        is_saturated = (unclamped_output >= self.out_max) or (unclamped_output <= self.out_min)

        integral_direction = np.sign(error * eff_kp)
        output_direction = np.sign(unclamped_output)
        is_pushing_harder = (integral_direction == output_direction)

        if not (is_saturated and is_pushing_harder):
            self.integral += error * dt

        I = eff_ki * self.integral

        # 5. Final Output Clamping
        final_output = np.clip(P + I + D, self.out_min, self.out_max)

        return final_output