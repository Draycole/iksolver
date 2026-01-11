import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

L = 5   # length of each link

# ====== FORWARD KINEMATICS ======
def fk(a1, a2, L):
    a1r = np.radians(a1)

    a2r_global = np.radians(-a2)  # convert clockwise to CCW for calculation
    
    x1 = L * np.cos(a1r)
    y1 = L * np.sin(a1r)

    # second link is at global angle a2r_global not relative
    x2 = x1 + L * np.cos(a2r_global)
    y2 = y1 + L * np.sin(a2r_global)

    return (0, x1, x2), (0, y1, y2)

# graph setup
fig, ax = plt.subplots()
plt.subplots_adjust(left=0.1, bottom=0.35)

# default angles
a1_init, a2_init = 30, 0
xline, yline = fk(a1_init, a2_init, L)
(robot_line,) = ax.plot(xline, yline, marker="o", linewidth=3)

ax.set_xlim(-2*L, 2*L)
ax.set_ylim(-2*L, 2*L)
ax.set_aspect("equal")
ax.grid(True, which='both', linestyle='-', alpha=0.7)
ticks = np.arange(-2*L, 2*L + 1, 1)
ax.set_xticks(ticks)
ax.set_yticks(ticks)
ax.minorticks_off()
ax.set_title("2-Link Robot Arm - FK/IK Visualization")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# sliders
ax_a1 = plt.axes([0.2, 0.2, 0.65, 0.03])
ax_a2 = plt.axes([0.2, 0.15, 0.65, 0.03])
a1_slider = Slider(ax_a1, 'Alpha1', 0, 180, valinit=a1_init)
a2_slider = Slider(ax_a2, 'Alpha2', -180, 180, valinit=a2_init)

# update
def update(val):
    a1 = a1_slider.val
    a2 = a2_slider.val
    xline, yline = fk(a1, a2, L)
    robot_line.set_xdata(xline)
    robot_line.set_ydata(yline)
    
    # title 
    end_x, end_y = xline[2], yline[2]
    ax.set_title(f"2-Link Robot Arm - End Effector: ({end_x:.2f}, {end_y:.2f})")
    
    fig.canvas.draw_idle()

a1_slider.on_changed(update)
a2_slider.on_changed(update)

update(None)

plt.show()