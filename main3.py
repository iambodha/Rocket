import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.linalg import inv
from scipy.signal import butter, filtfilt

def butter_lowpass(cutoff, fs, order=5):
    """Design a lowpass butterworth filter"""
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def apply_lowpass_filter(data, cutoff, fs, order=5):
    """Apply lowpass filter to the data"""
    b, a = butter_lowpass(cutoff, fs, order=order)
    return filtfilt(b, a, data)

def calculate_angles(accel_x, accel_y, accel_z):
    """Calculate roll and pitch angles from accelerometer data"""
    roll = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2))
    pitch = -np.arctan2(accel_x, np.sqrt(accel_y**2 + accel_z**2))
    return roll, pitch

def calculate_vertical_accel(accel_x, accel_y, accel_z, pitch, roll):
    """Calculate vertical acceleration in inertial frame"""
    accel_z_inertial = (-np.sin(pitch) * accel_x + 
                        np.cos(pitch) * np.sin(roll) * accel_y +
                        np.cos(pitch) * np.cos(roll) * accel_z)
    return (accel_z_inertial - 1) * 9.81

class KalmanFilter:
    def __init__(self, dt):
        self.dt = dt
        self.F = np.array([[1, dt],
                          [0, 1]])
        self.G = np.array([[0.5 * dt**2],
                          [dt]])
        self.H = np.array([[0, 1]])
        self.Q = self.G @ self.G.T * 0.1
        self.R = np.array([[1.0]])
        self.P = np.zeros((2, 2))
        self.x = np.zeros((2, 1))
    
    def update(self, accel):
        self.x = self.F @ self.x + self.G * accel
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.P = self.P - self.P @ self.H.T @ inv(self.H @ self.P @ self.H.T + self.R) @ self.H @ self.P
        return self.x[1, 0]

# Read the CSV file
df = pd.read_csv('testData.csv')

# Calculate time step and sampling frequency
dt = (df['Time(ms)'].iloc[1] - df['Time(ms)'].iloc[0]) / 1000.0
fs = 1/dt

# Initialize Kalman filter
kf = KalmanFilter(dt)

# Lists to store results
raw_accelerations = []
filtered_accelerations = []
velocities = []
times = df['Time(ms)'].values

# Process each row
for _, row in df.iterrows():
    roll, pitch = calculate_angles(row['AccelX'], row['AccelY'], row['AccelZ'])
    accel_z_inertial = calculate_vertical_accel(
        row['AccelX'], row['AccelY'], row['AccelZ'], pitch, roll
    )
    raw_accelerations.append(accel_z_inertial)

# Apply lowpass filter to accelerations
filtered_accelerations = apply_lowpass_filter(raw_accelerations, cutoff=2.0, fs=fs)

# Calculate velocities using filtered accelerations
for accel in filtered_accelerations:
    velocity = kf.update(accel)
    velocities.append(velocity)

# Create three subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))

# Plot velocity
ax1.plot(times, velocities, 'b-', label='Vertical Velocity', linewidth=2)
ax1.set_xlabel('Time (ms)')
ax1.set_ylabel('Vertical Velocity (m/s)')
ax1.set_title('Estimated Vertical Velocity from IMU Data')
ax1.grid(True)
ax1.legend()

# Plot both raw and filtered acceleration
ax2.plot(times, raw_accelerations, 'r:', label='Raw Acceleration', alpha=0.5)
ax2.plot(times, filtered_accelerations, 'g-', label='Filtered Acceleration', linewidth=2)
ax2.set_xlabel('Time (ms)')
ax2.set_ylabel('Vertical Acceleration (m/s²)')
ax2.set_title('Vertical Acceleration from IMU Data')
ax2.grid(True)
ax2.legend()

# Plot raw accelerometer data
ax3.plot(times, df['AccelX'], 'r-', label='X', linewidth=1.5)
ax3.plot(times, df['AccelY'], 'g-', label='Y', linewidth=1.5)
ax3.plot(times, df['AccelZ'], 'b-', label='Z', linewidth=1.5)
ax3.set_xlabel('Time (ms)')
ax3.set_ylabel('Acceleration (g)')
ax3.set_title('Raw Accelerometer Data')
ax3.grid(True)
ax3.legend()

plt.tight_layout()
plt.show()

# Print some statistics
print(f"Average vertical velocity: {np.mean(velocities):.3f} m/s")
print(f"Maximum vertical velocity: {np.max(velocities):.3f} m/s")
print(f"Minimum vertical velocity: {np.min(velocities):.3f} m/s")

# Print accelerometer statistics
print("\nAccelerometer Statistics (in g):")
print("\nX-axis:")
print(f"Average: {np.mean(df['AccelX']):.3f}")
print(f"Max: {np.max(df['AccelX']):.3f}")
print(f"Min: {np.min(df['AccelX']):.3f}")

print("\nY-axis:")
print(f"Average: {np.mean(df['AccelY']):.3f}")
print(f"Max: {np.max(df['AccelY']):.3f}")
print(f"Min: {np.min(df['AccelY']):.3f}")

print("\nZ-axis:")
print(f"Average: {np.mean(df['AccelZ']):.3f}")
print(f"Max: {np.max(df['AccelZ']):.3f}")
print(f"Min: {np.min(df['AccelZ']):.3f}")