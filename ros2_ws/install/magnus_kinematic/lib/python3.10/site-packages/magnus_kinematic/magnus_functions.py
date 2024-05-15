import numpy as np

L = 120
r = 48.5
def adjust_platform(pitch_deg, roll_deg, z):
    # Convert degrees to radians
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)
    yaw = np.radians(25)
    z = 20  # Heave

    # Pitch transformation matrix (Y-axis rotation)
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])

    # Roll transformation matrix (X-axis rotation)
    R_roll = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1]
    ])

    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1]
    ])

    # Heave transformation matrix (Z-axis translation)
    T_heave = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    # Overall transformation matrix
    Pprh = R_pitch @ R_roll @ R_yaw @ T_heave
    # Convert points to homogeneous coordinates
    p1 = np.array([L/2, L/(2*np.sqrt(3)), 0, 1])
    p2 = np.array([-L/2, L/(2*np.sqrt(3)), 0, 1])
    p3 = np.array([0, -L/(np.sqrt(3)), 0, 1])
    # Apply transformation
    p1_new = Pprh @ p1
    p2_new = Pprh @ p2
    p3_new = Pprh @ p3
    # Calculate servo angles
    servo1angle = np.rad2deg(np.arctan2(p1_new[2], r))
    servo2angle = np.rad2deg(np.arctan2(p2_new[2], r))
    servo3angle = np.rad2deg(np.arctan2(p3_new[2], r))
    return servo1angle, servo2angle, servo3angle



maxX = 50
minX = -50
maxY = 50
minY = -50

maxAngle = 45
minAngle = -45
offset = 0.4

def coordinates2angle(x,y):
    #print(f"x:{x}, y: {y}")
    roll = -y*offset
    pitch = -x*offset
    return pitch, roll # Pitch is around y, Roll is around x


















def PID(target, curpos, dt, prev_e, integral, max_buildup, kp, ki, kd):
    
    e = curpos - target 
    P = kp*e

    integral += e * dt


    if integral > max_buildup:
        integral = max_buildup
    
    I = ki * integral

    derivative = (e - prev_e) / dt
    D = kd * derivative 
    output = P + I + D    
    return output, e, integral