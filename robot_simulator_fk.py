import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

# DH 변환 행렬


def dh_transform(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),
         np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -
         np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),
         np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

# Forward Kinematics


def forward_kinematics(thetas, a, d, alpha):
    T = np.eye(4)
    positions = [T[:3, 3]]  # 원점 포함
    for i, theta in enumerate(thetas):
        T = T @ dh_transform(theta, d[i], a[i], alpha[i])
        positions.append(T[:3, 3])
    return np.array(positions)


# 초기 DH 파라미터 (예시)
a = [0, -0.425, -0.392, 0,    0,    0]
d = [0.089, 0,      0,      0.109, 0.095, 0.082]
alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
init_thetas = [0, 0, 0, 0, 0, 0]

# 3D 시각화 함수


def plot_robot(thetas, ax, line, txt):
    pos = forward_kinematics(thetas, a, d, alpha)
    xs, ys, zs = pos[:, 0], pos[:, 1], pos[:, 2]
    line.set_data(xs, ys)
    line.set_3d_properties(zs)
    txt.set_text(f"EE: x={xs[-1]:.3f}, y={ys[-1]:.3f}, z={zs[-1]:.3f}")
    ax.relim()
    ax.autoscale_view()
    plt.draw()


# Figure & Axes
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.35)

# 초기 로봇 그리기
pos0 = forward_kinematics(init_thetas, a, d, alpha)
line, = ax.plot(pos0[:, 0], pos0[:, 1], pos0[:, 2], '-o', lw=2)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Interactive 6-DOF FK Simulator")

# 슬라이더 축들 생성
slider_axes = []
sliders = []
for i in range(6):
    ax_si = fig.add_axes([0.1, 0.25 - i*0.03, 0.8, 0.02])
    slider_axes.append(ax_si)
    sliders.append(
        Slider(ax_si, f"θ{i+1}", -np.pi, np.pi, valinit=init_thetas[i])
    )

# 슬라이더 콜백 등록


def update(_):
    thetas = [s.val for s in sliders]
    plot_robot(thetas, ax, line, txt)


for s in sliders:
    s.on_changed(update)

# Reset 버튼
reset_ax = fig.add_axes([0.8, 0.02, 0.1, 0.04])
button = Button(reset_ax, 'Reset')


def reset(event):
    for s, init in zip(sliders, init_thetas):
        s.reset()


button.on_clicked(reset)

plt.show()
