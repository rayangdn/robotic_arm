import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Reuse the forward kinematics function
def forward_kinematics_2_dof(q1, q2, l1, l2):
    q1 = np.deg2rad(q1)
    q2 = np.deg2rad(q2)
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y

# Inverse kinematics to calculate joint angles from end-effector position
def inverse_kinematics_2_dof(x, y, l1, l2):
    d = np.sqrt(x**2 + y**2)
    if d > l1 + l2 or d < abs(l1 - l2):
        # No solution (out of reach)
        return None, None
    
    q2 = np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
    q1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    
    return np.rad2deg(q1), np.rad2deg(q2)

# Set up the plot
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.25)

# Arm link lengths
l1, l2 = 120, 90

# Initial end-effector position
x_init, y_init = forward_kinematics_2_dof(45, 45, l1, l2)

# Create the plot elements
link1, = ax.plot([0, l1 * np.cos(np.deg2rad(45))], [0, l1 * np.sin(np.deg2rad(45))], 'r-', lw=3)
link2, = ax.plot([l1 * np.cos(np.deg2rad(45)), x_init], [l1 * np.sin(np.deg2rad(45)), y_init], 'b-', lw=3)
end_effector, = ax.plot(x_init, y_init, 'ko', markersize=10)

# Set the plot limits and labels
ax.set_xlim(-250, 250)
ax.set_ylim(-250, 250)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_title('2-DOF Robotic Arm Simulation')

# Create sliders for end-effector position
ax_x = plt.axes([0.1, 0.1, 0.8, 0.03])
ax_y = plt.axes([0.1, 0.05, 0.8, 0.03])

slider_x = Slider(ax_x, 'X', -250, 250, valinit=x_init)
slider_y = Slider(ax_y, 'Y', -250, 250, valinit=y_init)

# Text to display q1 and q2 values
text_q1_q2 = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='blue')

# Update function for the plot
def update(val):
    # Get the end-effector position from sliders
    x = slider_x.val
    y = slider_y.val
    
    # Calculate joint angles using inverse kinematics
    q1, q2 = inverse_kinematics_2_dof(x, y, l1, l2)
    
    if q1 is None or q2 is None:
        # If no solution, display error message and return
        text_q1_q2.set_text("Out of reach")
        return
    
    # Update link positions using forward kinematics
    x1 = l1 * np.cos(np.deg2rad(q1))
    y1 = l1 * np.sin(np.deg2rad(q1))
    x2 = x1 + l2 * np.cos(np.deg2rad(q1 + q2))
    y2 = y1 + l2 * np.sin(np.deg2rad(q1 + q2))
    
    link1.set_data([0, x1], [0, y1])
    link2.set_data([x1, x2], [y1, y2])
    end_effector.set_data(x2, y2)
    
    # Update the text to display the current values of q1 and q2
    text_q1_q2.set_text(f'q1: {q1:.2f}°, q2: {q2:.2f}°')
    
    fig.canvas.draw_idle()

# Connect the sliders to the update function
slider_x.on_changed(update)
slider_y.on_changed(update)

# Initial call to update to set the initial values of q1 and q2
update(None)

plt.show()
