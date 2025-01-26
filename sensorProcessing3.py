import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('flightData.csv')

# Extract necessary columns
time = data['Time(ms)'].values
accel_x = data['AccelX(g)'].values
accel_y = data['AccelY(g)'].values
accel_z = data['AccelZ(g)'].values
gyro_x = data['GyroX(rad/s)'].values
gyro_y = data['GyroY(rad/s)'].values
gyro_z = data['GyroZ(rad/s)'].values
pressure = data['Pressure(Pa)'].values

# Constants
dt = 0.004  # Time step
g = 9.81  # Gravity
altitude_threshold = 100  # Threshold for significant altitude change in cm

# Kalman filter matrices
F = np.array([[1, dt], [0, 1]])
G = np.array([[0.5 * dt**2], [dt]])
H = np.array([[1, 0]])
I = np.eye(2)
Q = G @ G.T * 10**2
R = np.array([[30**2]])
P = np.zeros((2, 2))
S = np.zeros((2, 1))

# Initialize variables
altitude_kalman = []
velocity_vertical_kalman = []
altitude_barometer_startup = np.mean(44330 * (1 - (pressure[:2000] / 101325)**(1/5.255)) * 100)
significant_change_detected = False

# Kalman filter function
def kalman_2d(acc_z_inertial, altitude_barometer):
    global S, P
    Acc = np.array([[acc_z_inertial]])
    S = F @ S + G @ Acc
    P = F @ P @ F.T + Q
    L = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(L)
    M = np.array([[altitude_barometer]])
    S = S + K @ (M - H @ S)
    P = (I - K @ H) @ P
    return S[0, 0], S[1, 0]

# Process data
for i in range(len(time)):
    angle_roll = np.arctan2(accel_y[i], np.sqrt(accel_x[i]**2 + accel_z[i]**2)) * (180 / np.pi)
    angle_pitch = -np.arctan2(accel_x[i], np.sqrt(accel_y[i]**2 + accel_z[i]**2)) * (180 / np.pi)
    acc_z_inertial = (-np.sin(np.radians(angle_pitch)) * accel_x[i] +
                      np.cos(np.radians(angle_pitch)) * np.sin(np.radians(angle_roll)) * accel_y[i] +
                      np.cos(np.radians(angle_pitch)) * np.cos(np.radians(angle_roll)) * accel_z[i])
    acc_z_inertial = (acc_z_inertial - 1) * g * 100
    altitude_barometer = 44330 * (1 - (pressure[i] / 101325)**(1/5.255)) * 100 - altitude_barometer_startup
    altitude, velocity = kalman_2d(acc_z_inertial, altitude_barometer)
    
    if not significant_change_detected and abs(altitude) > altitude_threshold:
        significant_change_detected = True
        print(f"Significant altitude change detected at time: {time[i]} ms")
    
    if not significant_change_detected:
        velocity = 0
    
    altitude_kalman.append(altitude)
    velocity_vertical_kalman.append(velocity)

# Plot the altitude and vertical velocity
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(time, altitude_kalman)
plt.xlabel('Time (ms)')
plt.ylabel('Altitude (cm)')
plt.title('Altitude over Time')

plt.subplot(2, 1, 2)
plt.plot(time, velocity_vertical_kalman)
plt.xlabel('Time (ms)')
plt.ylabel('Vertical Velocity (cm/s)')
plt.title('Vertical Velocity over Time')

plt.tight_layout()
plt.show()
