import csv
import numpy as np
import plotly.graph_objects as go
from math import sqrt
from datetime import datetime
import os

acc_Bias = 0.255+0.976-0.21+0.41-0.23+0.04+0.27-0.9+0.13+0.53-0.05+0.17-0.32+0.19-0.1+0.45-0.34
altitude_Bias = 100.39+93+0.5-98
class MadgwickAHRS:
    def __init__(self, sample_freq=72.7, beta=0.1):
        self.sample_freq = sample_freq
        self.beta = beta
        # Quaternion of sensor frame relative to auxiliary frame
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(self, gx, gy, gz, ax, ay, az, mx, my, mz):
        """
        Update orientation using magnetometer, accelerometer and gyroscope data.
        All inputs should be in appropriate units:
        - Gyroscope: radians/second
        - Accelerometer: g (gravitational acceleration)
        - Magnetometer: gauss
        """
        # Fast inverse square-root
        def inv_sqrt(x):
            return 1.0 / sqrt(x)

        # Use IMU algorithm if magnetometer measurement invalid
        if (mx == 0.0) and (my == 0.0) and (mz == 0.0):
            return self.update_imu(gx, gy, gz, ax, ay, az)

        # Rate of change of quaternion from gyroscope
        qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        # Compute feedback only if accelerometer measurement valid
        if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
            # Normalize accelerometer measurement
            recip_norm = inv_sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            # Normalize magnetometer measurement
            recip_norm = inv_sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

            # Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0 * self.q0 * mx
            _2q0my = 2.0 * self.q0 * my
            _2q0mz = 2.0 * self.q0 * mz
            _2q1mx = 2.0 * self.q1 * mx
            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _2q0q2 = 2.0 * self.q0 * self.q2
            _2q2q3 = 2.0 * self.q2 * self.q3
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * self.q3 + _2q0mz * self.q2 + mx * q1q1 + _2q1 * my * self.q2 + _2q1 * mz * self.q3 - mx * q2q2 - mx * q3q3
            hy = _2q0mx * self.q3 + my * q0q0 - _2q0mz * self.q1 + _2q1mx * self.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.q3 - my * q3q3
            _2bx = sqrt(hx * hx + hy * hy)
            _2bz = -_2q0mx * self.q2 + _2q0my * self.q1 + mz * q0q0 + _2q1mx * self.q3 - mz * q1q1 + _2q2 * my * self.q3 - mz * q2q2 + mz * q3q3
            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz

            # Gradient descent algorithm corrective step
            s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q3 + _2bz * self.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q2 + _2bz * self.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q3 - _4bz * self.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.q2 - _2bz * self.q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q1 + _2bz * self.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q0 - _4bz * self.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.q3 + _2bz * self.q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q0 + _2bz * self.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
            
            # Normalize step magnitude
            recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            s0 *= recip_norm
            s1 *= recip_norm
            s2 *= recip_norm
            s3 *= recip_norm

            # Apply feedback step
            qDot1 -= self.beta * s0
            qDot2 -= self.beta * s1
            qDot3 -= self.beta * s2
            qDot4 -= self.beta * s3

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += qDot1 * (1.0 / self.sample_freq)
        self.q1 += qDot2 * (1.0 / self.sample_freq)
        self.q2 += qDot3 * (1.0 / self.sample_freq)
        self.q3 += qDot4 * (1.0 / self.sample_freq)

        # Normalize quaternion
        recip_norm = inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm

        return [self.q0, self.q1, self.q2, self.q3]

    def update_imu(self, gx, gy, gz, ax, ay, az):
            """
            Update orientation using only accelerometer and gyroscope data.
            """
            # Fast inverse square-root
            def inv_sqrt(x):
                return 1.0 / sqrt(x)

            # Rate of change of quaternion from gyroscope
            qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
            qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
            qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
            qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

            # Compute feedback only if accelerometer measurement valid
            if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):
                # Normalize accelerometer measurement
                recip_norm = inv_sqrt(ax * ax + ay * ay + az * az)
                ax *= recip_norm
                ay *= recip_norm
                az *= recip_norm

                # Auxiliary variables to avoid repeated arithmetic
                _2q0 = 2.0 * self.q0
                _2q1 = 2.0 * self.q1
                _2q2 = 2.0 * self.q2
                _2q3 = 2.0 * self.q3
                _4q0 = 4.0 * self.q0
                _4q1 = 4.0 * self.q1
                _4q2 = 4.0 * self.q2
                _8q1 = 8.0 * self.q1
                _8q2 = 8.0 * self.q2
                q0q0 = self.q0 * self.q0
                q1q1 = self.q1 * self.q1
                q2q2 = self.q2 * self.q2
                q3q3 = self.q3 * self.q3

                # Gradient decent algorithm corrective step
                s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
                s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
                s2 = 4.0 * q0q0 * self.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
                s3 = 4.0 * q1q1 * self.q3 - _2q1 * ax + 4.0 * q2q2 * self.q3 - _2q2 * ay

                # Normalize step magnitude
                recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
                s0 *= recip_norm
                s1 *= recip_norm
                s2 *= recip_norm
                s3 *= recip_norm

                # Apply feedback step
                qDot1 -= self.beta * s0
                qDot2 -= self.beta * s1
                qDot3 -= self.beta * s2
                qDot4 -= self.beta * s3

            # Integrate rate of change of quaternion to yield quaternion
            self.q0 += qDot1 * (1.0 / self.sample_freq)
            self.q1 += qDot2 * (1.0 / self.sample_freq)
            self.q2 += qDot3 * (1.0 / self.sample_freq)
            self.q3 += qDot4 * (1.0 / self.sample_freq)

            # Normalize quaternion
            recip_norm = inv_sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
            self.q0 *= recip_norm
            self.q1 *= recip_norm
            self.q2 *= recip_norm
            self.q3 *= recip_norm

            return [self.q0, self.q1, self.q2, self.q3]

    def get_euler_angles(self):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        Returns angles in degrees
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
        cosr_cosp = 1.0 - 2.0 * (self.q1 * self.q1 + self.q2 * self.q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (self.q0 * self.q2 - self.q3 * self.q1)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (self.q0 * self.q3 + self.q1 * self.q2)
        cosy_cosp = 1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        return roll_deg, pitch_deg, yaw_deg

# Global variable to store the data
data = {}

def read_csv_data(file_path):
    global data
    with open(file_path, mode='r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            time = row["Time(ms)"]
            data[time] = {
                "AccelX(g)": row["AccelX(g)"],
                "AccelY(g)": row["AccelY(g)"],
                "AccelZ(g)": row["AccelZ(g)"],
                "GyroX(rad/s)": row["GyroX(rad/s)"],
                "GyroY(rad/s)": row["GyroY(rad/s)"],
                "GyroZ(rad/s)": row["GyroZ(rad/s)"],
                "MagnX(gauss)": row["MagnX(gauss)"],
                "MagnY(gauss)": row["MagnY(gauss)"],
                "MagnZ(gauss)": row["MagnZ(gauss)"],
                "MPUTemp(C)": row["MPUTemp(C)"],
                "BaroTemp(C)": row["BaroTemp(C)"],
                "Pressure(Pa)": row["Pressure(Pa)"]
            }

def calculate_altitude(baro_temp, pressure, reference_pressure=101325):
    temp_kelvin = baro_temp + 273.15
    R = 287.05
    g = 9.80665
    altitude = ((temp_kelvin / 0.0065) * (1 - (pressure / reference_pressure) ** (R * 0.0065 / g)))+altitude_Bias
    return altitude

def get_altitude_for_time(time):
    if time in data:
        baro_temp = float(data[time]["BaroTemp(C)"])
        pressure = float(data[time]["Pressure(Pa)"])
        return calculate_altitude(baro_temp, pressure)
    else:
        return None

def plot_altitude():
    times = sorted(data.keys(), key=lambda x: int(x))
    altitudes = [get_altitude_for_time(time) for time in times]

    fig = go.Figure()

    fig.add_trace(go.Scatter(
        x=times,
        y=altitudes,
        mode='lines+markers',
        name='Altitude over Time'
    ))

    fig.update_layout(
        title='Altitude vs Time',
        xaxis_title='Time (ms)',
        yaxis_title='Altitude (m)',
        xaxis=dict(
            tickmode='linear',
            tick0=0,
            dtick=10  # Adjust this value to show fewer timestamps
        ),
        yaxis=dict(
            tickmode='linear',
            tick0=0,
            dtick=10  # Adjust this value to show fewer altitudes
        ),
        showlegend=True
    )

    fig.show()
    def smooth_data(data, window_size=5):
        smoothed_data = []
        for i in range(len(data)):
            start = max(0, i - window_size // 2)
            end = min(len(data), i + window_size // 2 + 1)
            window = data[start:end]
            smoothed_data.append(np.mean(window))
        return smoothed_data

    def plot_smoothed_altitude():
        times = sorted(data.keys(), key=lambda x: int(x))
        altitudes = [get_altitude_for_time(time) for time in times]
        smoothed_altitudes = smooth_data(altitudes)

        fig = go.Figure()

        fig.add_trace(go.Scatter(
            x=times,
            y=altitudes,
            mode='lines+markers',
            name='Raw Altitude'
        ))

        fig.add_trace(go.Scatter(
            x=times,
            y=smoothed_altitudes,
            mode='lines+markers',
            name='Smoothed Altitude'
        ))

        fig.update_layout(
            title='Altitude vs Time',
            xaxis_title='Time (ms)',
            yaxis_title='Altitude (m)',
            xaxis=dict(
                tickmode='linear',
                tick0=0,
                dtick=10  # Adjust this value to show fewer timestamps
            ),
            yaxis=dict(
                tickmode='linear',
                tick0=0,
                dtick=10  # Adjust this value to show fewer altitudes
            ),
            showlegend=True
        )

        fig.show()

    plot_smoothed_altitude()

madgwick = MadgwickAHRS(sample_freq=50, beta=0.1)

def process_imu_data(time):
    if time in data:
        # Get IMU data from your data dictionary
        gx = float(data[time]["GyroX(rad/s)"])
        gy = float(data[time]["GyroY(rad/s)"])
        gz = float(data[time]["GyroZ(rad/s)"])
        ax = float(data[time]["AccelX(g)"])
        ay = float(data[time]["AccelY(g)"])
        az = float(data[time]["AccelZ(g)"])
        mx = float(data[time]["MagnX(gauss)"])
        my = float(data[time]["MagnY(gauss)"])
        mz = float(data[time]["MagnZ(gauss)"])

        # Update orientation
        madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz)
        
        # Get Euler angles
        roll, pitch, yaw = madgwick.get_euler_angles()
        return roll, pitch, yaw
    return None, None, None

def export_flight_data(data):
    """
    Export flight data including altitude, smoothed altitude, and vertical acceleration to a CSV file.
    
    Parameters:
    data (dict): Dictionary containing the flight data
    
    Returns:
    str: Path to the saved CSV file
    """
    current_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f'flight_analysis.csv'
    
    # Get sorted timestamps
    times = sorted(data.keys(), key=lambda x: int(x))
    
    # Normalize timestamps by subtracting the first timestamp
    base_time = int(times[0])
    normalized_times = [int(time) - base_time for time in times]
    
    # Calculate all the required data
    altitudes = [get_altitude_for_time(time) for time in times]
    smoothed_altitudes = smooth_data(altitudes)
    vertical_accs = [calculate_vertical_acceleration(time) for time in times]
    
    # Write data to CSV
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        # Write header
        writer.writerow(['Time(ms)', 'Altitude(m)', 'Smoothed_Altitude(m)', 'Vertical_Acceleration(g)'])
        
        # Write data rows
        for i, time in enumerate(normalized_times):
            writer.writerow([
                time,
                altitudes[i] if altitudes[i] is not None else '',
                smoothed_altitudes[i] if smoothed_altitudes[i] is not None else '',
                vertical_accs[i] if vertical_accs[i] is not None else ''
            ])
    
    return os.path.abspath(filename)

def smooth_data(data, window_size=5):
    """
    Smooth data using a moving average.
    """
    smoothed_data = []
    for i in range(len(data)):
        start = max(0, i - window_size // 2)
        end = min(len(data), i + window_size // 2 + 1)
        window = [x for x in data[start:end] if x is not None]
        if window:
            smoothed_data.append(np.mean(window))
        else:
            smoothed_data.append(None)
    return smoothed_data

def plot_orientation():
    times = sorted(data.keys(), key=lambda x: int(x))
    orientations = [process_imu_data(time) for time in times]
    
    roll = [o[0] for o in orientations if o[0] is not None]
    pitch = [o[1] for o in orientations if o[1] is not None]
    yaw = [o[2] for o in orientations if o[2] is not None]
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(x=times, y=roll, mode='lines', name='Roll'))
    fig.add_trace(go.Scatter(x=times, y=pitch, mode='lines', name='Pitch'))
    fig.add_trace(go.Scatter(x=times, y=yaw, mode='lines', name='Yaw'))
    
    fig.update_layout(
        title='Orientation vs Time',
        xaxis_title='Time (ms)',
        yaxis_title='Angle (degrees)',
        showlegend=True
    )
    
    fig.show()

def calculate_vertical_acceleration(time):
    if time in data:
        # Get orientation angles
        roll, pitch, yaw = process_imu_data(time)
        
        if roll is None:
            return None
            
        # Convert angles to radians
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        
        # Get accelerometer readings in g
        ax = float(data[time]["AccelX(g)"])
        ay = float(data[time]["AccelY(g)"])
        az = float(data[time]["AccelZ(g)"])
        
        # Create rotation matrix (simplified for vertical acceleration)
        # We only need the third row since we're only interested in vertical component
        R31 = np.sin(pitch_rad)
        R32 = -np.cos(pitch_rad) * np.sin(roll_rad)
        R33 = np.cos(pitch_rad) * np.cos(roll_rad)
        
        # Calculate vertical acceleration in Earth frame
        vertical_acc = ax * R31 + ay * R32 + az * R33
        
        # Subtract 1g to remove gravity and negate the result so positive is upward
        vertical_acc = -((vertical_acc - 1.0) + acc_Bias)
        
        return vertical_acc
    return None

def plot_vertical_acceleration():
    times = sorted(data.keys(), key=lambda x: int(x))
    vertical_accs = [calculate_vertical_acceleration(time) for time in times]
    
    # Remove None values
    valid_times = []
    valid_accs = []
    for t, acc in zip(times, vertical_accs):
        if acc is not None:
            valid_times.append(t)
            valid_accs.append(acc)
    
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=valid_times,
        y=valid_accs,
        mode='lines',
        name='Vertical Acceleration'
    ))
    
    fig.update_layout(
        title='Vertical Acceleration vs Time',
        xaxis_title='Time (ms)',
        yaxis_title='Acceleration (g)',
        showlegend=True
    )
    
    fig.show()

# Read data from CSV file
read_csv_data('flightData.csv')

# Get altitude for a specific time
time = "5000"
altitude = get_altitude_for_time(time)

plot_altitude()
plot_vertical_acceleration()

# Export the flight data to CSV
exported_file = export_flight_data(data)
print(f"Flight data exported to: {exported_file}")

