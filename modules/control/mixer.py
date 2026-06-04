"""
Control Mixer.
Maps logical commands (Pitch/Roll/Yaw Cmd) to physical actuators (Elevator/Aileron/Rudder).
Supports Standard, Delta Wing, and V-Tail configurations.
Includes output saturation protection.

控制混控器。
将逻辑指令映射到物理舵面。
支持常规布局、三角翼和 V 尾布局。
包含输出饱和保护。
"""
import numpy as np
from modules.dynamics.forces import ControlInputs

class Mixer:
    def __init__(self, mixer_type="standard"):
        self.type = mixer_type

    def mix(self, pitch_cmd: float, roll_cmd: float, yaw_cmd: float, throttle_cmd: float) -> ControlInputs:
        """
        Args:
            pitch_cmd: -1.0 to 1.0
            roll_cmd:  -1.0 to 1.0
            yaw_cmd:   -1.0 to 1.0
            throttle_cmd: 0.0 to 1.0
        """
        controls = ControlInputs()

        # Throttle is usually direct pass-through
        controls.throttle = throttle_cmd

        # --- Mixing Logic / 混控逻辑 ---
        if self.type == "standard":
            # Conventional layout (Cessna, F-16)
            controls.elevator = pitch_cmd
            controls.aileron = roll_cmd
            controls.rudder = yaw_cmd

        elif self.type == "delta":
            # Delta wing (Mirage, Concorde) - Elevons
            controls.elevator = pitch_cmd + roll_cmd # Left Elevon
            controls.aileron = pitch_cmd - roll_cmd  # Right Elevon
            controls.rudder = yaw_cmd

        elif self.type == "v_tail":
            # V-Tail (Bonanza, Predator) - Ruddervators
            controls.elevator = pitch_cmd + yaw_cmd  # Left Ruddervator
            controls.rudder = pitch_cmd - yaw_cmd    # Right Ruddervator
            controls.aileron = roll_cmd

        # --- FIX: Secondary Clipping (Actuator Saturation) ---
        # 修复：二次限幅 (执行机构饱和保护)
        # Ensures that mixed commands never exceed physical limits [-1.0, 1.0]
        # 确保混控后的指令永远不会超过物理极限 [-1.0, 1.0]
        controls.elevator = float(np.clip(controls.elevator, -1.0, 1.0))
        controls.aileron = float(np.clip(controls.aileron, -1.0, 1.0))
        controls.rudder = float(np.clip(controls.rudder, -1.0, 1.0))
        controls.throttle = float(np.clip(controls.throttle, 0.0, 1.0))

        return controls