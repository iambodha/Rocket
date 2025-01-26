import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Step 1: Load the data from a CSV file
data = pd.read_csv("imu_data.csv")

# Step 2: Complementary Filter for sensor fusion (Accelerometer + Gyroscope)
def complementary_filter(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt=0.02, alpha=0.98):
    # Accelerometer calculations for pitch and roll
    roll_acc = np.arctan2(accel_y, accel_z)
    pitch_acc = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2))

    # Gyroscope integration for angular change
    roll_gyro = gyro_x * dt
    pitch_gyro = gyro_y * dt

    # Complementary filter to combine accelerometer and gyroscope
    roll = alpha * (roll_gyro) + (1 - alpha) * roll_acc
    pitch = alpha * (pitch_gyro) + (1 - alpha) * pitch_acc

    return roll, pitch

# Step 3: Function to remove gravity from the accelerometer data
def remove_gravity(accel_x, accel_y, accel_z):
    # Calculate the gravity vector (assuming the accelerometer is aligned with gravity when at rest)
    g = np.array([0, 0, 9.81])  # Gravitational acceleration in m/s^2 (assuming Earth gravity)
    
    # Normalize the accelerometer data to remove gravity
    accel_vector = np.array([accel_x, accel_y, accel_z])
    gravity_projection = np.dot(accel_vector, g) / np.dot(g, g) * g  # Projection of accel on gravity
    accel_no_gravity = accel_vector - gravity_projection  # Subtract gravity component

    return accel_no_gravity

# Step 4: Initialize arrays to store filtered orientation data, velocities, and heights
rolls = []
pitches = []
accel_x_no_gravity = []
accel_y_no_gravity = []
accel_z_no_gravity = []
velocities = []
heights = []

# Initial conditions (assuming the IMU starts at rest at height 0)
velocity_z = 0  # Initial vertical velocity (m/s)
height = 0  # Initial height (m)

# Loop through the data to apply the complementary filter, gravity removal, and compute velocity/height
for i in range(len(data)):
    accel_x, accel_y, accel_z = data.iloc[i][['AccelX', 'AccelY', 'AccelZ']]
    gyro_x, gyro_y, gyro_z = data.iloc[i][['GyroX', 'GyroY', 'GyroZ']]
    
    # Remove gravity from accelerometer data
    accel_no_gravity = remove_gravity(accel_x, accel_y, accel_z)
    
    # Store the cleaned accelerometer data (no gravity)
    accel_x_no_gravity.append(accel_no_gravity[0])
    accel_y_no_gravity.append(accel_no_gravity[1])
    accel_z_no_gravity.append(accel_no_gravity[2])
    
    # Apply the complementary filter to get roll and pitch
    roll, pitch = complementary_filter(accel_x_no_gravity[-1], accel_y_no_gravity[-1], accel_z_no_gravity[-1], gyro_x, gyro_y, gyro_z)
    rolls.append(roll)
    pitches.append(pitch)
    
    # Step 5: Integrate acceleration to get velocity (using simple integration)
    # Assuming each timestep (dt) is constant, we integrate acceleration to get velocity
    dt = 0.02  # Timestep in seconds (assuming 50 Hz sampling rate)
    velocity_z += accel_z_no_gravity[-1] * dt  # Integrate to get velocity
    
    # Step 6: Integrate velocity to get height (displacement)
    height += velocity_z * dt  # Integrate velocity to get displacement (height)
    
    # Store the computed height
    heights.append(height)

# Add the filtered data to the dataframe
data['Roll'] = rolls
data['Pitch'] = pitches
data['AccelX_no_gravity'] = accel_x_no_gravity
data['AccelY_no_gravity'] = accel_y_no_gravity
data['AccelZ_no_gravity'] = accel_z_no_gravity
data['Height'] = heights

# Step 7: Visualization - Plot the height over time
plt.figure(figsize=(10, 6))
plt.plot(data['Time(ms)'], data['Height'], label='Height (m)')
plt.xlabel('Time (ms)')
plt.ylabel('Height (m)')
plt.legend()
plt.title('Height vs Time')
plt.show()

# Step 8: Accelerometer Data Plot (No Gravity)
plt.figure(figsize=(10, 6))
plt.plot(data['Time(ms)'], data['AccelX_no_gravity'], label='AccelX (No Gravity)')
plt.plot(data['Time(ms)'], data['AccelY_no_gravity'], label='AccelY (No Gravity)')
plt.plot(data['Time(ms)'], data['AccelZ_no_gravity'], label='AccelZ (No Gravity)')
plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s²)')
plt.legend()
plt.title('Accelerometer Data (No Gravity)')
plt.show()

# Step 9: Plot the Roll and Pitch angles over time
plt.figure(figsize=(10, 6))
plt.plot(data['Time(ms)'], np.degrees(data['Roll']), label='Roll (degrees)')
plt.plot(data['Time(ms)'], np.degrees(data['Pitch']), label='Pitch (degrees)')
plt.xlabel('Time (ms)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.title('Roll and Pitch Angles over Time')
plt.show()
