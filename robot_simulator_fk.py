import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button


# dht
def dh_transform(theta, d, a, alpha):
    """Create the transformation matrix based on DH parameters."""
    return np.array(
        [
            [
                np.cos(theta),
                -np.sin(theta) * np.cos(alpha),
                np.sin(theta) * np.sin(alpha),
                a * np.cos(theta),
            ],
            [
                np.sin(theta),
                np.cos(theta) * np.cos(alpha),
                -np.cos(theta) * np.sin(alpha),
                a * np.sin(theta),
            ],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1],
        ]
    )


# FK
def forward_kinematics(theta_list, a, d, alpha):
    """Calculate the forward kinematics to get end effector position."""
    T = np.eye(4)
    positions = []
    for i in range(len(theta_list)):
        T_i = dh_transform(theta_list[i], d[i], a[i], alpha[i])
        T = T @ T_i  # Combine transformations
        positions.append(T[:3, 3])  # Append the position of the end effector
    return T, positions


# 축 범위 지정
def init_axes(ax):
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 1.4)


# 변수 정의
a = [0, -0.425, -0.392, 0, 0, 0]  # 링크 길이
d = [0.089, 0, 0, 0.109, 0.095, 0.082]  # 링크 옵셋
alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]  # 링크축 사이의 회전각
init_thetas = [0, 0, 0, 0, 0, 0]

# figure setting
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection="3d")
plt.subplots_adjust(bottom=0.35)

ax.view_init(elev=25, azim=45)  # 카메라 시점
ax.dist = 12  # 카메라 거리
init_axes(ax)

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("3D visualization of Robotic Arm")

# plot robot
init_ee_pos = forward_kinematics(init_thetas, a, d, alpha)
line = ax.plot(init_ee_pos[:, 0], init_ee_pos[:, 1], init_ee_pos[:, 2], "-o", lw=2)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

# sliders
sliders = []
for i in range(6):
    slider = fig.add_axes([0.35, 0.25 - i * 0.03, 0.3, 0.02])
    sliders.append(Slider(slider, f"Theta {i+1}", -np.pi, np.pi, valinit=0))


def update(_):
    thetas = [s.val for s in sliders]
    T, ee_pos = forward_kinematics(thetas, a, d, alpha)
    line.set_data(ee_pos[:, 0], ee_pos[:, 1])
    line.set_3d_properties(ee_pos[:, 2])
    txt.set_text(
        f"EE: x={ee_pos[-1, 0]:.3f}, y={ee_pos[-1, 1]:.3f}, z={ee_pos[-1, 2]:.3f}"
    )
    fig.canvas.draw_idle()


for s in sliders:
    s.on_changed(update)
    
plt.show()
