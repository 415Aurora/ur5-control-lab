from __future__ import annotations

import math

import numpy as np


def parse_pose_target_values(
    x_text: str,
    y_text: str,
    z_text: str,
    roll_text: str,
    pitch_text: str,
    yaw_text: str,
) -> tuple[float, float, float, float, float, float]:
    try:
        return (
            float(x_text),
            float(y_text),
            float(z_text),
            float(roll_text),
            float(pitch_text),
            float(yaw_text),
        )
    except ValueError:
        raise ValueError("Pose target must be valid X/Y/Z/Roll/Pitch/Yaw numbers")


def quaternion_from_euler_degrees(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[float, float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def euler_degrees_from_rotation_matrix(
    rotation: np.ndarray,
) -> tuple[float, float, float]:
    sy = math.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
