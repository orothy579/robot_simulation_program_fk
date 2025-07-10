import numpy as np
import matplotlib.pyplot as plt


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


def visualize_robot(pos):
    pos = np.array(pos)
    xs, ys, zs = pos[:, 0], pos[:, 1], pos[:, 2]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs, ys, zs, '-o', linewidth=2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title("3D Visualization of Robotic Arm")
    plt.show()


theta = [0, 0, 0, 0, 0, 0]
a = [0, -0.425, -0.392, 0, 0, 0]
d = [0.089, 0, 0, 0.109, 0.095, 0.082]
alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]

T, ee_pos = forward_kinematics(theta, a, d, alpha)
visualize_robot(ee_pos)
