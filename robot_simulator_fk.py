import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

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

# axes limit setting
def init_axes(ax):
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(0, 1.4)

# from 3x3 rot mat to euler
def to_euler(mat):
    r = R.from_matrix(mat)
    return r.as_euler('xyz', degrees=True)  # roll, pitch, yaw


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
T, init_ee_pos = forward_kinematics(init_thetas, a, d, alpha)
init_ee_pos = np.array(init_ee_pos)
line = ax.plot(init_ee_pos[:, 0], init_ee_pos[:, 1], init_ee_pos[:, 2], "-o", lw=2)[0]
start_marker = ax.scatter(init_ee_pos[0, 0], init_ee_pos[0, 1], init_ee_pos[0, 2], color="red", marker="o", s=50)
end_marker = ax.scatter(
    init_ee_pos[-1, 0], init_ee_pos[-1, 1], init_ee_pos[-1, 2],
    color="green", marker="o", s=50
)
txt = ax.text2D(0.25, 0.95, "", transform=ax.transAxes)

# sliders
sliders = []
for i in range(6):
    slider = fig.add_axes([0.35, 0.25 - i * 0.03, 0.3, 0.02])
    sliders.append(Slider(slider, f"Theta {i+1}", -180, 180, valinit=0, valfmt="%1.1f"))


def update(_):
    deg_thetas = [s.val for s in sliders]
    rad_thetas = np.radians(deg_thetas)
    T, ee_pos = forward_kinematics(rad_thetas, a, d, alpha)
    ee_pos = np.array(ee_pos)
    line.set_data(ee_pos[:,0], ee_pos[:,1])
    line.set_3d_properties(ee_pos[:,2])
    
    end_marker._offsets3d = (
        [ee_pos[-1, 0]],
        [ee_pos[-1, 1]],
        [ee_pos[-1, 2]]
    )
    
    rot_mat = T[:3, :3]
    r,p,y = to_euler(rot_mat)

    txt.set_text(
        f"Pos: x={ee_pos[-1, 0]:.3f}, y={ee_pos[-1, 1]:.3f}, z={ee_pos[-1, 2]:.3f}\n"
        f"Ori: roll={r:.1f}, pitch={p:.1f}, yaw={y:.1f}"
    )
    fig.canvas.draw_idle()


for s in sliders:
    s.on_changed(update)
    
plt.show()
