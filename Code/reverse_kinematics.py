import numpy as np

# 2 DOFs Kinematics

# Forwards Kinematics

def forward_kinematics_2_dof(q1, q2, l1, l2):

    q1 = np.deg2rad(q1)
    q2 = np.deg2rad(q2)

    x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)

    return x, y

# Inverse Kinematics

def inverse_kinematics_2_dof(x, y, l1, l2):

    r = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)

    q2 = np.arccos((r**2 - l1**2 - l2**2) / (2 * l1 * l2))
    q1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))

    q1 = np.rad2deg(q1)
    q2 = np.rad2deg(q2)

    return q1, q2


# 3 DOF Kinematics

# Forwards Kinematics

def forward_kinematics_3_dof(q1, q2, q3, l1, l2, l3):
    
        q1 = np.deg2rad(q1)
        q2 = np.deg2rad(q2)
        q3 = np.deg2rad(q3)
    
        x = l1 * np.cos(q1) + l2 * np.cos(q1 + q2) + l3 * np.cos(q1 + q2 + q3)
        y = l1 * np.sin(q1) + l2 * np.sin(q1 + q2) + l3 * np.sin(q1 + q2 + q3)
        theta = q1 + q2 + q3

        theta = np.rad2deg(theta)
    
        return x, y, theta

# Inverse Kinematics

def inverse_kinematics_3_dof(x, y, theta, l1, l2, l3):

    theta = np.deg2rad(theta)
    
    # Calculate the position for the 2-DOF subsystem
    x_2dof = x - l3 * np.cos(theta)
    y_2dof = y - l3 * np.sin(theta)
    
    q1, q2 = inverse_kinematics_2_dof(x_2dof, y_2dof, l1, l2)
    q3 = theta - np.deg2rad(q1) - np.deg2rad(q2)

    return q1, q2, np.rad2deg(q3)

def is_reachable(x, y, theta, l1, l2, l3):
    r = np.sqrt(x**2 + y**2)
    x_2dof = x - l3 * np.cos(theta)
    y_2dof = y - l3 * np.sin(theta)

    r_2dof = np.sqrt(x_2dof**2 + y_2dof**2)

    if(r > (l1 + l2 + l3) or r < abs(l1 - l2 - l3)):
        return False
    if(r_2dof > l1 + l2 or r_2dof < abs(l1 - l2)):
        return False
    return True
    
l1, l2, l3 = 120, 90, 20
x, y, theta = 50, 50, 20

q1, q2 = inverse_kinematics_2_dof(x, y, l1, l2)
print("Inverse Kinematics 2 DOFs")
print(f"q1: {q1:.2f}, q2: {q2:.2f}")

x, y = forward_kinematics_2_dof(q1, q2, l1, l2)
print("Forward Kinematics 2 DOFs")
print(f"x: {x:.2f}, y: {y:.2f}")

if(is_reachable(x, y, theta, l1, l2, l3)):
    q1, q2, q3 = inverse_kinematics_3_dof(x, y, theta, l1, l2, l3)
    print("Inverse Kinematics 3 DOFs")
    print(f"q1: {q1:.2f}, q2: {q2:.2f}, q3: {q3:.2f}")

    x, y, theta = forward_kinematics_3_dof(q1, q2, q3, l1, l2, l3)
    print("Forward Kinematics 3 DOFs")
    print(f"x: {x:.2f}, y: {y:.2f}, theta: {theta:.2f}")
else:
    print(f"Error in 3-DOF inverse kinematics")