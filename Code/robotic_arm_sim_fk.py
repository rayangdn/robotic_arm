import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Reuse the forward kinematics function from the original code
def forward_kinematics_2_dof(q1, q2, l1, l2):
    q1 = np.deg2rad(q1)
    q2 = np.deg2rad(q2)
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y

# Set up the plot
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.25)

# Initial parameters
l1, l2 = 120, 90
q1_init, q2_init = 45, 45

# Calculate initial end effector position
x, y = forward_kinematics_2_dof(q1_init, q2_init, l1, l2)

# Create the plot elements
link1, = ax.plot([0, l1 * np.cos(np.deg2rad(q1_init))], [0, l1 * np.sin(np.deg2rad(q1_init))], 'r-', lw=3)
link2, = ax.plot([l1 * np.cos(np.deg2rad(q1_init)), x], [l1 * np.sin(np.deg2rad(q1_init)), y], 'b-', lw=3)
end_effector, = ax.plot(x, y, 'ko', markersize=10)

# Set the plot limits and labels
ax.set_xlim(-250, 250)
ax.set_ylim(-250, 250)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_title('2-DOF Robotic Arm Simulation')

# Create sliders for joint angles
ax_q1 = plt.axes([0.1, 0.1, 0.8, 0.03])
ax_q2 = plt.axes([0.1, 0.05, 0.8, 0.03])

slider_q1 = Slider(ax_q1, 'q1', -180, 180, valinit=q1_init)
slider_q2 = Slider(ax_q2, 'q2', -180, 180, valinit=q2_init)

# Update function for the plot
def update(val):
    q1 = slider_q1.val
    q2 = slider_q2.val
    x, y = forward_kinematics_2_dof(q1, q2, l1, l2)
    
    link1.set_data([0, l1 * np.cos(np.deg2rad(q1))], [0, l1 * np.sin(np.deg2rad(q1))])
    link2.set_data([l1 * np.cos(np.deg2rad(q1)), x], [l1 * np.sin(np.deg2rad(q1)), y])
    end_effector.set_data(x, y)
    
    fig.canvas.draw_idle()

# Connect the sliders to the update function
slider_q1.on_changed(update)
slider_q2.on_changed(update)

plt.show()