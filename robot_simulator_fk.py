# 예시 코드
import numpy as np


def dh_transform(theta, d, a, alpha):
    """Create the transformation matrix based on DH parameters."""
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha),
         np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta),  np.cos(theta) * np.cos(alpha), -
         np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0,              np.sin(alpha),                   np.cos(
            alpha),                  d],
        [0,              0,
            0,                             1]
    ])


def forward_kinematics(theta_list, a, d, alpha):
    """Calculate the forward kinematics to get end effector position."""
    T = np.eye(4)
    positions = []
    for i in range(len(theta_list)):
        T_i = dh_transform(theta_list[i], d[i], a[i], alpha[i])
        T = T @ T_i  # Combine transformations
        positions.append(T[:3, 3])  # Append the position of the end effector
    return T, positions
