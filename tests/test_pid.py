"""
Unit tests for PID Controller.
Verifies proportional logic, integration, and output saturation.

PID 控制器的单元测试。
验证比例逻辑、积分和输出饱和。
"""

import unittest
import sys
import os

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.control.pid import PID


class TestPID(unittest.TestCase):

    def setUp(self):
        # Initialize a standard PID before each test
        # 在每次测试前初始化一个标准 PID
        self.pid = PID(kp=1.0, ki=0.1, kd=0.0, out_min=-10.0, out_max=10.0)

    def test_proportional_logic(self):
        """
        Test basic P-term calculation.
        测试基本 P 项计算。
        """
        # Setpoint=10, Meas=0 -> Error=10
        # Output = Kp * Error = 1.0 * 10 = 10.0
        output = self.pid.update(setpoint=10.0, measurement=0.0, dt=0.1)
        self.assertEqual(output, 10.0)

    def test_saturation_limits(self):
        """
        Test if output is clamped to min/max limits.
        测试输出是否被限制在最大/最小值范围内。
        """
        # Huge error: 1000.0
        # Theoretical output: 1000.0
        # Limit: 10.0
        output = self.pid.update(setpoint=1000.0, measurement=0.0, dt=0.1)

        self.assertEqual(output, 10.0)  # Should be clamped to max

    def test_integral_accumulation(self):
        """
        Test if integral term accumulates over time.
        测试积分项是否随时间累积。
        """
        # Step 1: Error = 1.0, dt = 1.0
        # P = 1.0, I = 0.1 * 1.0 = 0.1 -> Out = 1.1
        out1 = self.pid.update(1.0, 0.0, 1.0)
        self.assertAlmostEqual(out1, 1.1)

        # Step 2: Error = 1.0, dt = 1.0
        # P = 1.0, I = 0.1 + (0.1 * 1.0) = 0.2 -> Out = 1.2
        out2 = self.pid.update(1.0, 0.0, 1.0)
        self.assertAlmostEqual(out2, 1.2)


if __name__ == '__main__':
    unittest.main()