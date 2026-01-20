"""
Control Mixer.
Maps logical commands (Pitch/Roll/Yaw Cmd) to physical actuators (Elevator/Aileron/Rudder).
Supports Standard, Delta Wing, and V-Tail configurations.

控制混控器。
将逻辑指令映射到物理舵面。
支持常规布局、三角翼和 V 尾布局。
"""
from modules.dynamics.forces import ControlInputs


class Mixer:
    def __init__(self, mixer_type="standard"):
        self.type = mixer_type

    def mix(self, pitch_cmd, roll_cmd, yaw_cmd, throttle_cmd):
        controls = ControlInputs()
        controls.throttle = throttle_cmd

        if self.type == "standard":
            controls.elevator = pitch_cmd
            controls.aileron = roll_cmd
            controls.rudder = yaw_cmd
        elif self.type == "delta":
            controls.elevator = pitch_cmd + roll_cmd
            controls.aileron = pitch_cmd - roll_cmd
            controls.rudder = yaw_cmd
        elif self.type == "v_tail":
            controls.elevator = pitch_cmd + yaw_cmd
            controls.rudder = pitch_cmd - yaw_cmd
            controls.aileron = roll_cmd

        return controls