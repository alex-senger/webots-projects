import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# Arm segment lengths

L1, L2 = 2.0, 1.5


def inverse_kinematics(x, y):
    """Calculate joint angles (theta1, theta2) to reach (x, y)"""

    D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(D) > 1:  # point is out of reach
        return None, None

    theta2 = np.arccos(D)

    theta1 = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(theta2), L1 + L2 * np.cos(theta2)
    )

    return theta1, theta2


def forward_kinematics(theta1, theta2):
    """Calculate joint positions from angles"""

    x1 = L1 * np.cos(theta1)

    y1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)

    y2 = y1 + L2 * np.sin(theta1 + theta2)

    return [0, x1, x2], [0, y1, y2]


fig, ax = plt.subplots()

(line,) = ax.plot([], [], "o-", lw=4)

(target_dot,) = ax.plot([], [], "rx", markersize=10)


def init():

    ax.set_xlim(-4, 4)

    ax.set_ylim(-4, 4)

    return line, target_dot


def update(frame):

    return line, target_dot


def on_click(event):

    x, y = event.xdata, event.ydata

    target_dot.set_data([x], [y])

    theta1, theta2 = inverse_kinematics(x, y)

    if theta1 is not None:
        x_vals, y_vals = forward_kinematics(theta1, theta2)

        line.set_data(x_vals, y_vals)

    else:
        print("Target is out of reach!")


fig.canvas.mpl_connect("button_press_event", on_click)

ani = FuncAnimation(fig, update, init_func=init, blit=True)

plt.title("Click to Move the 2-Link Arm")

plt.grid()

plt.show()
