import numpy as np
import serial
import time
import pygame

THRESHOLD = 0.5  # Joystick deadzone threshold
# Initialize serial communication with ESP32
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port accordingly
time.sleep(2)  # Wait for connection to establish

# Initialize Pygame for controller input
pygame.init()
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    pygame.quit()
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Arm link lengths
l1, l2 = 120, 90
q1, q2, q3, q4, q5 = 45, -40, 0, -15, 0  # Initial joint angles including q5

# Constants
THRESHOLD = 0.1  # Define a threshold for detecting significant joystick movement

# Function to update joint angles based on joystick input and button presses
def update_joint_angles(q1, q2, q3, q4, q5, l1_button, r1_button, left_stick_x, x_button, s_button, right_stick_y, t_button, o_button):

    # Shoulder control (q1)
    if l1_button:
        q1 += 5
        if q1 > 180:  # Limit lower bound for shoulder
            q1 = 180
    elif r1_button:
        q1 -= 5
        if q1 < 0:  # Limit upper bound for shoulder
            q1 = 0

    # Elbow control (q2)
    if left_stick_x > THRESHOLD:
        q2 -= 5
        if q2 > 90:  # Limit upper bound for elbow
            q2 = 90
    elif left_stick_x < -THRESHOLD:
        q2 += 5
        if q2 < -90:  # Limit lower bound for elbow
            q2 = -90

    # Wrist roll control (q3)
    if x_button:  # X button pressed
        q3 += 5  # Clockwise movement
        if q3 > 180:
            q3 = 180  # Limit the maximum angle
    elif s_button:  # O button pressed
        q3 -= 5  # Anticlockwise movement
        if q3 < 0:
            q3 = 0  # Limit the minimum angle

    # Wrist pitch control (q4)
    if right_stick_y > THRESHOLD:
        q4 -= 5
        if q4 < -90:  # Limit lower bound for wrist pitch
            q4 = -90
    elif right_stick_y < -THRESHOLD:
        q4 += 5
        if q4 > 90:  # Limit upper bound for wrist pitch
            q4 = 90

    # Gripper control (q5)
    if t_button:
        q5 += 20
        if q5 > 90:
            q5 = 90
    elif o_button:
        q5 -= 5
        if q5 < 0:
            q5 = 0

    return q1, q2, q3, q4, q5

# Main control loop
try:
    while True:
        # Handle Pygame events
        pygame.event.pump()

        # Get joystick axes (left stick for shoulder/elbow, right stick for wrist)
        left_stick_x = joystick.get_axis(0)  # For shoulder
        left_stick_y = joystick.get_axis(1)  # For elbow
        right_stick_x = joystick.get_axis(2)  # For wrist roll
        right_stick_y = joystick.get_axis(3)  # For wrist pitch

        # Get button states
        x_button = joystick.get_button(0)  # X button 
        t_button = joystick.get_button(2)  # Triangle button
        o_button = joystick.get_button(1)  # Circle button
        s_button = joystick.get_button(3)  # Square button 
        l1_button = joystick.get_button(4)
        r1_button = joystick.get_button(5)


        # Update joint angles based on joystick input and button presses
        q1, q2, q3, q4, q5 = update_joint_angles(q1, q2, q3, q4, q5, l1_button, r1_button, left_stick_x, x_button, s_button, right_stick_y, t_button, o_button)

        # Send joint angles to ESP32
        try:
            ser.write(f"{q1},{q2},{q3},{q4}, {q5}\n".encode())
            print(f"q1: {q1:.2f}, q2: {q2:.2f}, q3: {q3:.2f}, q4: {q4:.2f}, q5: {q5:.2f}")
            time.sleep(0.01)
        except Exception as e:
            print(f"Error sending data to ESP32: {e}")

        # Delay to match controller update rate
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # Close serial communication and quit pygame
    ser.close()
    pygame.quit()
