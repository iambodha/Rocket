import pandas as pd
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt

# Helper functions moved to top level
def quaternion_to_rotation_matrix(q):
    """Convert quaternion to rotation matrix"""
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
        [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])

def pressure_to_altitude(pressure, base_pressure):
    """Convert pressure to altitude using barometric formula"""
    return 44330 * (1 - (pressure/base_pressure)**(1/5.255))

class AttitudeEstimator:
    def __init__(self):
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
        self.beta = 0.1  # Gradient descent step size
        
    def update(self, accel, gyro, mag, dt):
        """
        Update attitude estimation using accelerometer, gyroscope, and magnetometer
        Based on Madgwick's MARG implementation
        """
        # Normalize measurements
        if np.linalg.norm(accel) == 0 or np.linalg.norm(mag) == 0:
            return self.q
            
        accel = accel / np.linalg.norm(accel)
        mag = mag / np.linalg.norm(mag)
        
        # Reference direction of Earth's magnetic field
        h = self.quaternion_multiply(
            self.quaternion_multiply(self.q, [0, mag[0], mag[1], mag[2]]),
            self.quaternion_conjugate(self.q)
        )
        b = [0, np.sqrt(h[1]**2 + h[2]**2), 0, h[3]]
        
        # Gradient descent algorithm corrective step
        F = np.array([
            2*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - accel[0],
            2*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - accel[1],
            2*(0.5 - self.q[1]**2 - self.q[2]**2) - accel[2],
            2*b[1]*(0.5 - self.q[2]**2 - self.q[3]**2) + 2*b[3]*(self.q[1]*self.q[3] - self.q[0]*self.q[2]) - mag[0],
            2*b[1]*(self.q[1]*self.q[2] - self.q[0]*self.q[3]) + 2*b[3]*(self.q[0]*self.q[1] + self.q[2]*self.q[3]) - mag[1],
            2*b[1]*(self.q[0]*self.q[2] + self.q[1]*self.q[3]) + 2*b[3]*(0.5 - self.q[1]**2 - self.q[2]**2) - mag[2]
        ])
        
        J = np.array([
            [-2*self.q[2], 2*self.q[3], -2*self.q[0], 2*self.q[1]],
            [2*self.q[1], 2*self.q[0], 2*self.q[3], 2*self.q[2]],
            [0, -4*self.q[1], -4*self.q[2], 0],
            [-2*b[3]*self.q[2], 2*b[3]*self.q[3], -4*b[1]*self.q[2]-2*b[3]*self.q[0], -4*b[1]*self.q[3]+2*b[3]*self.q[1]],
            [-2*b[1]*self.q[3]+2*b[3]*self.q[1], 2*b[1]*self.q[2]+2*b[3]*self.q[0], 2*b[1]*self.q[1]+2*b[3]*self.q[3], -2*b[1]*self.q[0]+2*b[3]*self.q[2]],
            [2*b[1]*self.q[2], 2*b[1]*self.q[3]-4*b[3]*self.q[1], 2*b[1]*self.q[0]-4*b[3]*self.q[2], 2*b[1]*self.q[1]]
        ])
        
        step = J.T @ F
        step = step / np.linalg.norm(step)
        
        # Compute rate of change of quaternion
        qDot = 0.5 * self.quaternion_multiply(self.q, [0, gyro[0], gyro[1], gyro[2]]) - self.beta * step
        
        # Integrate to yield quaternion
        self.q = self.q + qDot * dt
        self.q = self.q / np.linalg.norm(self.q)
        
        return self.q
    
    @staticmethod
    def quaternion_multiply(a, b):
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
        ])
    
    @staticmethod
    def quaternion_conjugate(q):
        return np.array([q[0], -q[1], -q[2], -q[3]])

class RocketKalmanFilter:
    def __init__(self):
        # State vector: [position, velocity, acceleration]
        self.state = np.zeros(3)
        
        # State transition matrix
        self.F = np.eye(3)
        
        # Initial state covariance
        self.P = np.eye(3) * 1000
        
        # Process noise covariance
        self.Q = np.eye(3) * 0.1
        
        # Measurement noise covariance
        self.R_accel = 5.0  # Accelerometer noise
        self.R_baro = 2.0   # Barometer noise
    
    def predict(self, dt):
        # Update state transition matrix
        self.F[0, 1] = dt
        self.F[0, 2] = 0.5 * dt * dt
        self.F[1, 2] = dt
        
        # Predict state
        self.state = self.F @ self.state
        
        # Predict covariance
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update_accel(self, accel_measurement):
        H = np.array([[0, 0, 1]])  # Measurement matrix for accelerometer
        self._update(accel_measurement, H, self.R_accel)
    
    def update_baro(self, baro_measurement):
        H = np.array([[1, 0, 0]])  # Measurement matrix for barometer
        self._update(baro_measurement, H, self.R_baro)
    
    def _update(self, measurement, H, R):
        # Innovation
        y = measurement - H @ self.state
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T / S
        
        # Update state
        self.state += K @ y
        
        # Update covariance
        self.P = (np.eye(3) - K @ H) @ self.P

def process_flight_data(df):
    """Process flight data with attitude estimation and Kalman filtering"""
    data = df.copy()
    
    # Convert time to seconds
    data['Time'] = data['Time(ms)'] / 1000.0
    data['dt'] = data['Time'].diff().fillna(0)
    
    # Initialize estimators
    attitude = AttitudeEstimator()
    kf = RocketKalmanFilter()
    
    # Arrays to store results
    earth_accels = []
    kf_positions = []
    kf_velocities = []
    kf_accelerations = []
    
    # Process each timestep
    for i in range(len(data)):
        dt = data['dt'].iloc[i]
        
        # Get sensor readings
        accel = np.array([
            data['AccelX(g)'].iloc[i],
            data['AccelY(g)'].iloc[i],
            data['AccelZ(g)'].iloc[i]
        ])
        
        gyro = np.array([
            data['GyroX(rad/s)'].iloc[i],
            data['GyroY(rad/s)'].iloc[i],
            data['GyroZ(rad/s)'].iloc[i]
        ])
        
        mag = np.array([
            data['MagnX(gauss)'].iloc[i],
            data['MagnY(gauss)'].iloc[i],
            data['MagnZ(gauss)'].iloc[i]
        ])
        
        # Update attitude estimation
        q = attitude.update(accel, gyro, mag, dt)
        
        # Convert acceleration to Earth frame
        accel_body = accel * 9.81
        R = quaternion_to_rotation_matrix(q)  # Fixed: removed self reference
        accel_earth = R @ accel_body
        vertical_accel = accel_earth[2] - 9.81
        
        earth_accels.append(vertical_accel)
        
        # Kalman filter prediction and update
        if dt > 0:
            kf.predict(dt)
            kf.update_accel(vertical_accel)
            
            # Update with barometer if available
            if 'Pressure(Pa)' in data.columns:
                altitude = pressure_to_altitude(data['Pressure(Pa)'].iloc[i],
                                             data['Pressure(Pa)'].iloc[0])
                kf.update_baro(altitude)
        
        # Store Kalman filter estimates
        kf_positions.append(kf.state[0])
        kf_velocities.append(kf.state[1])
        kf_accelerations.append(kf.state[2])
    
    # Add results to dataframe
    data['Earth_Vertical_Acceleration'] = earth_accels
    data['KF_Position'] = kf_positions
    data['KF_Velocity'] = kf_velocities
    data['KF_Acceleration'] = kf_accelerations
    
    return data

def plot_results(data):
    """Create visualization of flight data"""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
    
    # Plot vertical acceleration
    ax1.plot(data['Time'], data['Earth_Vertical_Acceleration'],
             'b-', alpha=0.3, label='Raw (Earth Frame)')
    ax1.plot(data['Time'], data['KF_Acceleration'],
             'r-', label='Kalman Filtered')
    ax1.set_ylabel('Vertical Acceleration (m/s²)')
    ax1.set_title('Vertical Acceleration in Earth Frame')
    ax1.grid(True)
    ax1.legend()
    
    # Plot vertical velocity
    ax2.plot(data['Time'], data['KF_Velocity'],
             'g-', label='Kalman Filtered')
    ax2.set_ylabel('Vertical Velocity (m/s)')
    ax2.set_title('Vertical Velocity')
    ax2.grid(True)
    ax2.legend()
    
    # Plot altitude
    if 'Pressure(Pa)' in data.columns:
        baro_height = data.apply(
            lambda row: pressure_to_altitude(row['Pressure(Pa)'], 
                                          data['Pressure(Pa)'].iloc[0]),
            axis=1)
        ax3.plot(data['Time'], baro_height - baro_height.iloc[0],
                'b--', label='Barometer', alpha=0.5)
    
    ax3.plot(data['Time'], data['KF_Position'],
             'r-', label='Kalman Filtered')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Altitude (m)')
    ax3.set_title('Altitude')
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    return fig

def analyze_flight(csv_path):
    """Analyze rocket flight data"""
    df = pd.read_csv(csv_path)
    processed_data = process_flight_data(df)
    fig = plot_results(processed_data)
    return processed_data, fig
# Analyze flight data
data, figure = analyze_flight('flightData.csv')

# Display plots
plt.show()

# Access specific data
print(data[['Time', 'Earth_Vertical_Acceleration', 'KF_Acceleration', 'KF_Position']].head())
