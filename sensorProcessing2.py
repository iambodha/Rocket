import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from filterpy.kalman import KalmanFilter

# Read the data
def read_sensor_data(filename):
    return pd.read_csv(filename)

# Convert pressure to altitude using barometric formula
def pressure_to_altitude(pressure, temperature):
    P0 = 101325  # Standard pressure at sea level in Pa
    return 44330 * (1 - (pressure / P0)**(1 / 5.255))

# Calculate velocity from barometric altitude
def calculate_velocity_from_altitude(altitude, time):
    altitude = np.array(altitude)
    time = np.array(time)
    
    # Calculate differences in altitude and time
    delta_altitude = np.diff(altitude)
    delta_time = np.diff(time) / 1000  # Convert ms to seconds
    
    # Velocity calculation
    velocity = delta_altitude / delta_time
    
    # Prepend zero for initial velocity
    velocity = np.insert(velocity, 0, 0)  # Assuming initial velocity is 0
    return velocity

# Initialize Kalman Filter for sensor fusion
def init_kalman_filter():
    kf = KalmanFilter(dim_x=3, dim_z=2)  # State: [position, velocity, acceleration]
    dt = 0.02  # 20ms sampling rate
    
    # State transition matrix
    kf.F = np.array([[1, dt, 0.5 * dt**2],
                     [0, 1, dt],
                     [0, 0, 1]])
    
    # Measurement matrix
    kf.H = np.array([[1, 0, 0],
                     [0, 0, 1]])
    
    # Measurement noise
    kf.R = np.array([[0.1, 0],
                     [0, 0.1]])
    
    # Process noise
    kf.Q = np.eye(3) * 0.01
    
    # Initial state uncertainty
    kf.P *= 1000
    
    return kf

def process_data(data):
    # Calculate altitude from pressure
    baro_altitude = pressure_to_altitude(data['Pressure(Pa)'].values, data['BaroTemp(C)'].values)
    
    # Calculate barometric velocity
    baro_velocity = calculate_velocity_from_altitude(baro_altitude, data['Time(ms)'].values)
    
    # IMU data
    accel = np.column_stack((data['AccelX(g)'].values, 
                             data['AccelY(g)'].values, 
                             data['AccelZ(g)'].values)) * 9.81
    gyro = np.column_stack((data['GyroX(rad/s)'].values,
                            data['GyroY(rad/s)'].values,
                            data['GyroZ(rad/s)'].values))
    
    # Initialize arrays for storing results
    n_samples = len(data)
    vertical_accel = np.zeros(n_samples)
    vertical_vel = np.zeros(n_samples)
    filtered_altitude = np.zeros(n_samples)
    filtered_velocity = np.zeros(n_samples)
    filtered_accel = np.zeros(n_samples)
    
    # Initialize Kalman filter
    kf = init_kalman_filter()
    kf.x = np.array([[baro_altitude[0]], [0], [0]])  # Initial state
    
    # Process each timestep
    dt = 0.02  # 20ms
    for i in range(n_samples):
        # Update orientation using complementary filter
        if i > 0:
            R = Rotation.from_euler('xyz', gyro[i] * dt)
            accel_vertical = R.apply(accel[i])[2] - 9.81  # Remove gravity
            
            vertical_vel[i] = vertical_vel[i-1] + accel_vertical * dt
            vertical_accel[i] = accel_vertical
        
        # Kalman filter prediction and update
        kf.predict()
        z = np.array([[baro_altitude[i]], [vertical_accel[i]]])
        kf.update(z)
        
        filtered_altitude[i] = kf.x[0]
        filtered_velocity[i] = kf.x[1]
        filtered_accel[i] = kf.x[2]
    
    return (baro_altitude, baro_velocity, vertical_accel, vertical_vel,
            filtered_altitude, filtered_velocity, filtered_accel)

def plot_results(time, baro_alt, baro_vel, imu_accel, imu_vel,
                 filt_alt, filt_vel, filt_accel):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
    
    # Altitude plot
    ax1.plot(time, baro_alt, 'b-', label='Barometric Altitude')
    ax1.plot(time, filt_alt, 'r-', label='Filtered Altitude')
    ax1.set_ylabel('Altitude (m)')
    ax1.legend()
    ax1.grid(True)
    
    # Velocity plot
    ax2.plot(time, baro_vel, 'b-', label='Barometric Velocity')
    ax2.plot(time, imu_vel, 'g-', label='IMU Velocity')
    ax2.plot(time, filt_vel, 'r-', label='Filtered Velocity')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.legend()
    ax2.grid(True)
    
    # Acceleration plot
    ax3.plot(time, imu_accel, 'g-', label='IMU Acceleration')
    ax3.plot(time, filt_accel, 'r-', label='Filtered Acceleration')
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('Acceleration (m/s²)')
    ax3.legend()
    ax3.grid(True)
    
    plt.tight_layout()
    plt.show()

def main():
    # Read data
    data = read_sensor_data('flightData.csv')
    
    # User-specified time range
    time_range = (0, 100000000000)  # Change this to set a different range
    data = data[(data['Time(ms)'] >= time_range[0]) & (data['Time(ms)'] <= time_range[1])]
    
    # Process data
    results = process_data(data)
    baro_alt, baro_vel, imu_accel, imu_vel, filt_alt, filt_vel, filt_accel = results
    
    # Plot results
    plot_results(data['Time(ms)'].values, baro_alt, baro_vel, imu_accel, imu_vel,
                 filt_alt, filt_vel, filt_accel)
    
    # Print maximum values
    print(f"Maximum altitude: {np.max(filt_alt):.2f} m")
    print(f"Maximum velocity (barometric): {np.max(abs(baro_vel)):.2f} m/s")
    print(f"Maximum velocity (IMU): {np.max(abs(imu_vel)):.2f} m/s")
    print(f"Maximum velocity (filtered): {np.max(abs(filt_vel)):.2f} m/s")
    print(f"Maximum acceleration: {np.max(abs(filt_accel)):.2f} m/s²")

if __name__ == "__main__":
    main()
