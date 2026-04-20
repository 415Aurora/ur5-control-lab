from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src/ur5_control_lab/src"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from ur5_control_lab.core.pose_targets import (  # noqa: E402
    euler_degrees_from_rotation_matrix,
    parse_pose_target_values,
    quaternion_from_euler_degrees,
)


def test_parse_pose_target_values_requires_all_numeric_fields() -> None:
    with pytest.raises(ValueError, match="valid X/Y/Z/Roll/Pitch/Yaw numbers"):
        parse_pose_target_values("0.1", "0.2", "bad", "0", "0", "0")


def test_quaternion_from_euler_degrees_returns_identity_for_zero_rotation() -> None:
    qx, qy, qz, qw = quaternion_from_euler_degrees(0.0, 0.0, 0.0)
    assert qx == pytest.approx(0.0)
    assert qy == pytest.approx(0.0)
    assert qz == pytest.approx(0.0)
    assert qw == pytest.approx(1.0)


def test_euler_degrees_from_rotation_matrix_recovers_simple_yaw() -> None:
    yaw = math.radians(90.0)
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    roll_deg, pitch_deg, yaw_deg = euler_degrees_from_rotation_matrix(rotation)

    assert roll_deg == pytest.approx(0.0)
    assert pitch_deg == pytest.approx(0.0)
    assert yaw_deg == pytest.approx(90.0)
