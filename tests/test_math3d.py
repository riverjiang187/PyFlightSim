"""
Unit tests for MathUtils.
Verifies Quaternion <-> Euler conversions and vector operations.

MathUtils 的单元测试。
验证四元数与欧拉角的转换以及向量运算。
"""

import unittest
import numpy as np
import sys
import os

# Add project root to path to allow imports
# 将项目根目录添加到路径以允许导入
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from modules.utils.math3d import MathUtils


class TestMath3d(unittest.TestCase):

    def test_euler_quat_roundtrip(self):
        """
        Test if converting Euler -> Quat -> Euler returns original values.
        测试 欧拉角 -> 四元数 -> 欧拉角 是否能还原原始值。
        """
        # Test case: [Roll=0.1, Pitch=0.2, Yaw=0.3] rad
        original = np.array([0.1, 0.2, 0.3])

        # 1. Euler -> Quat
        q = MathUtils.euler_to_quat(original[0], original[1], original[2])

        # 2. Quat -> Euler
        result = MathUtils.quat_to_euler(q)

        # Check if close enough (floating point tolerance)
        # 检查是否足够接近 (浮点容差)
        np.testing.assert_array_almost_equal(original, result, decimal=6)

    def test_normalize_quat(self):
        """
        Test quaternion normalization.
        测试四元数归一化。
        """
        # Create a non-unit quaternion (Length = 2.0)
        # 创建一个非单位四元数 (长度 = 2.0)
        q_raw = np.array([2.0, 0.0, 0.0, 0.0])

        q_norm = MathUtils.normalize_quat(q_raw)

        # Magnitude should be 1.0
        # 模长应为 1.0
        self.assertAlmostEqual(np.linalg.norm(q_norm), 1.0)
        self.assertEqual(q_norm[0], 1.0)

    def test_quat_conjugate(self):
        """
        Test quaternion conjugate (inverse).
        测试四元数共轭 (逆)。
        """
        q = np.array([0.707, 0.0, 0.707, 0.0])
        q_conj = MathUtils.quat_conjugate(q)

        expected = np.array([0.707, -0.0, -0.707, -0.0])
        np.testing.assert_array_almost_equal(q_conj, expected)


if __name__ == '__main__':
    unittest.main()