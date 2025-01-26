import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import inv

def calculate_angles(accel_x, accel_y, accel_z):
    """Calculate roll and pitch angles from accelerometer data"""
    roll = np.arctan(accel_y / np.sqrt(accel_x**2 + accel_z**2))
    pitch = -np.arctan(accel_x / np.sqrt(accel_y**2 + accel_z**2))
    return roll, pitch

def calculate_vertical_accel(accel_x, accel_y, accel_z, pitch, roll):
    """Calculate vertical acceleration in inertial frame in m/s^2"""
    accel_z_inertial = (-np.sin(pitch) * accel_x + 
                        np.cos(pitch) * np.sin(roll) * accel_y +
                        np.cos(pitch) * np.cos(roll) * accel_z)
    return (accel_z_inertial - 1) * 9.81  # Convert to m/s^2

class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        # State transition matrix
        self.F = np.array([[1, dt],
                          [0, 1]])
        # Control input matrix
        self.G = np.array([[0.5 * dt**2],
                          [dt]])
        # Measurement matrix (we only measure velocity)
        self.H = np.array([[0, 1]])
        # Process noise covariance
        self.Q = self.G @ self.G.T * 25  # Tuned value
        # Measurement noise covariance
        self.R = np.array([[100]])  # Tuned value
        # Initial state covariance
        self.P = np.zeros((2, 2))
        # Initial state [position, velocity]
        self.x = np.zeros((2, 1))
    
    def update(self, accel):
        # Predict
        self.x = self.F @ self.x + self.G * accel
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        # Update covariance only (no position measurement)
        self.P = self.P - self.P @ self.H.T @ inv(self.H @ self.P @ self.H.T + self.R) @ self.H @ self.P
        
        return self.x[1, 0]  # Return velocity in m/s

# Read the CSV file
df = pd.read_csv('testData.csv')

# Calculate time step (dt) from the Time column
dt = (df['Time(ms)'].iloc[1] - df['Time(ms)'].iloc[0]) / 1000.0  # Convert to seconds for calculations

# Initialize Kalman filter
kf = KalmanFilter(dt)

# Lists to store results
velocities = []
accelerations = []
times = []

# Process each row
for _, row in df.iterrows():
    # Calculate angles
    roll, pitch = calculate_angles(row['AccelX'], row['AccelY'], row['AccelZ'])
    
    # Calculate vertical acceleration
    accel_z_inertial = calculate_vertical_accel(
        row['AccelX'], row['AccelY'], row['AccelZ'], pitch, roll
    )
    
    # Update Kalman filter and get velocity estimate
    velocity = kf.update(accel_z_inertial)
    
    velocities.append(velocity)
    accelerations.append(accel_z_inertial)
    times.append(row['Time(ms)'])

# Create two subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Plot velocity
ax1.plot(times, velocities, 'b-', label='Vertical Velocity')
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Vertical Velocity (m/s)')
ax1.set_title('Estimated Vertical Velocity from IMU Data')
ax1.grid(True)
ax1.legend()

# Plot acceleration
ax2.plot(times, accelerations, 'r-', label='Vertical Acceleration')
ax2.set_xlabel('Time (ms)')
ax2.set_ylabel('Vertical Acceleration (m/s²)')
ax2.set_title('Vertical Acceleration from IMU Data')
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()

# Print some statistics
print(f"Average vertical velocity: {np.mean(velocities):.3f} m/s")
print(f"Maximum vertical velocity: {np.max(velocities):.3f} m/s")
print(f"Minimum vertical velocity: {np.min(velocities):.3f} m/s")
print(f"\nAverage vertical acceleration: {np.mean(accelerations):.3f} m/s²")
print(f"Maximum vertical acceleration: {np.max(accelerations):.3f} m/s²")
print(f"Minimum vertical acceleration: {np.min(accelerations):.3f} m/s²")