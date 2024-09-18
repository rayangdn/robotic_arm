import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import serial
import time

# Open serial communication with the ESP32
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust the port if necessary
time.sleep(2)  # Wait for the connection to be established

# Forward kinematics function
def forward_kinematics_2_dof(q1, q2, l1, l2):
    q1 = np.deg2rad(q1)
    q2 = np.deg2rad(q2)
    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x, y

# Set up the plot
fig, ax = plt.subplots(figsize=(10, 8))
plt.subplots_adjust(left=0.1, bottom=0.35)

# Arm link lengths
l1, l2 = 120, 90
q1_init, q2_init, q3_init, q4_init = 0, 0, 0, 0

# Initial end-effector position
x_init, y_init = forward_kinematics_2_dof(q1_init, q2_init, l1, l2)

# Create the plot elements
link1, = ax.plot([0, l1 * np.cos(np.deg2rad(q1_init))], [0, l1 * np.sin(np.deg2rad(q1_init))], 'r-', lw=3)
link2, = ax.plot([l1 * np.cos(np.deg2rad(q1_init)), x_init], [l1 * np.sin(np.deg2rad(q1_init)), y_init], 'b-', lw=3)
end_effector, = ax.plot(x_init, y_init, 'ko', markersize=10)

# Set plot limits and labels
ax.set_xlim(-250, 250)
ax.set_ylim(-250, 250)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_title('4-DOF Robotic Arm Simulation')

# Create sliders for shoulder, elbow, wrist_roll, and wrist_pitch positions
ax_q1 = plt.axes([0.1, 0.25, 0.8, 0.03])
ax_q2 = plt.axes([0.1, 0.2, 0.8, 0.03])
ax_q3 = plt.axes([0.1, 0.15, 0.8, 0.03])
ax_q4 = plt.axes([0.1, 0.1, 0.8, 0.03])

slider_q1 = Slider(ax_q1, 'Shoulder', 0, 180, valinit=q1_init)
slider_q2 = Slider(ax_q2, 'Elbow', -90, 90, valinit=q2_init)
slider_q3 = Slider(ax_q3, 'Wrist Roll', 0, 180, valinit=q3_init)
slider_q4 = Slider(ax_q4, 'Wrist Pitch', -90, 90, valinit=q4_init)

# Text to display position values
text_x_y = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='blue')

# Update function for the plot and ESP32
def update(val):
    # Get the joint angles from sliders
    q1 = slider_q1.val
    q2 = slider_q2.val
    q3 = slider_q3.val
    q4 = slider_q4.val

    try:
        # Send data to ESP32
        ser.write(f"{q1},{q2},{q3},{q4}\n".encode())
        time.sleep(0.01)
        
        # Update forward kinematics for shoulder and elbow
        x, y = forward_kinematics_2_dof(q1, q2, l1, l2)
    except Exception as e:
        print(f"Error: {e}")
        return

    # Update link positions using forward kinematics
    x1 = l1 * np.cos(np.deg2rad(q1))
    y1 = l1 * np.sin(np.deg2rad(q1))
    x2 = x1 + l2 * np.cos(np.deg2rad(q1 + q2))
    y2 = y1 + l2 * np.sin(np.deg2rad(q1 + q2))
    
    link1.set_data([0, x1], [0, y1])
    link2.set_data([x1, x2], [y1, y2])
    end_effector.set_data(x2, y2)
    
    # Update the text to display the current values of x and y
    text_x_y.set_text(f'x: {x:.2f}, y: {y:.2f}')
    
    fig.canvas.draw_idle()

# Connect sliders to the update function
slider_q1.on_changed(update)
slider_q2.on_changed(update)
slider_q3.on_changed(update)
slider_q4.on_changed(update)

# Initial call to update to set the initial values
update(None)

plt.show()
