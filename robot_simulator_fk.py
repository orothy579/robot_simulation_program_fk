import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button

# ------------------ DH & FK ------------------


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


def forward_kinematics(thetas, a, d, alpha):
    T = np.eye(4)
    pos = [T[:3, 3]]
    for i, th in enumerate(thetas):
        T = T @ dh_transform(th, d[i], a[i], alpha[i])
        pos.append(T[:3, 3])
    return np.array(pos)


# ------------------ 파라미터 ------------------
a = [0, -0.425, -0.392, 0, 0, 0]
d = [0.089, 0, 0, 0.109, 0.095, 0.082]
alpha = [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]
init_thetas = [0]*6
BOUND = (abs(a[1]) + abs(a[2]) + d[0] + d[3] + d[4] + d[5]) * 1.2


# ------------------ 축 균일 함수 ------------------

def init_axes(ax):
    ax.set_xlim(-BOUND, BOUND)
    ax.set_ylim(-BOUND, BOUND)
    ax.set_zlim(0,  BOUND)        # 지면(0)~ +BOUND 정도로
    ax.set_box_aspect([1, 1, 1])    # 축 길이 동일


def set_axes_equal(ax, pad=1.2):
    xlim, ylim, zlim = ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()
    ranges = np.array([xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]])
    max_range = ranges.max()*pad
    mid = np.array([np.mean(xlim), np.mean(ylim), np.mean(zlim)])
    ax.set_xlim(mid[0]-max_range/2, mid[0]+max_range/2)
    ax.set_ylim(mid[1]-max_range/2, mid[1]+max_range/2)
    ax.set_zlim(mid[2]-max_range/2, mid[2]+max_range/2)


# ------------------ 초기 Figure ------------------
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(left=0.1, bottom=0.35)

# 시점 / 카메라 거리
ax.view_init(elev=25, azim=45)   # 바라보는 각도
ax.dist = 12                     # 카메라-타깃 거리(수치 ↑ → 더 멀리)
init_axes(ax)

# 로봇 초기 그리기
pos0 = forward_kinematics(init_thetas, a, d, alpha)
line, = ax.plot(pos0[:, 0], pos0[:, 1], pos0[:, 2], '-o', lw=2)
txt = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Interactive 6-DOF FK Simulator")
set_axes_equal(ax)               # 축 맞추고 패딩

# ------------------ 슬라이더 ------------------
sliders = []
for i in range(6):
    ax_s = fig.add_axes([0.1, 0.25 - i*0.03, 0.8, 0.02])
    sliders.append(Slider(ax_s, f"θ{i+1}", -np.pi, np.pi, valinit=0))


def update(_):
    thetas = [s.val for s in sliders]
    pos = forward_kinematics(thetas, a, d, alpha)
    line.set_data(pos[:, 0], pos[:, 1])
    line.set_3d_properties(pos[:, 2])
    txt.set_text(
        f"EE: x={pos[-1, 0]:.3f}, y={pos[-1, 1]:.3f}, z={pos[-1, 2]:.3f}")
    fig.canvas.draw_idle()


for s in sliders:
    s.on_changed(update)

# Reset 버튼
button_ax = fig.add_axes([0.8, 0.02, 0.1, 0.04])
button = Button(button_ax, 'Reset')
button.on_clicked(lambda e: [s.reset() for s in sliders])

plt.show()
