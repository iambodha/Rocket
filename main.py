import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R
from ahrs.filters import Madgwick

# Load IMU data from CSV
def load_imu_data(file_path):
    """
    Load IMU data from a CSV file.
    Expected columns: time, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
    """
    data = pd.read_csv(file_path)
    return data

# Transform accelerometer data to global frame using quaternion orientation
def transform_to_global(accel, quat):
    """
    Transform accelerometer readings from local frame to global frame.
    accel: numpy array [ax, ay, az] in local IMU frame
    quat: quaternion [w, x, y, z]
    """
    rotation = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # Convert to [x, y, z, w]
    return rotation.apply(accel)

# Subtract gravity to get linear acceleration
def subtract_gravity(accel_global):
    """Subtract gravity from global frame acceleration."""
    gravity = np.array([0, 0, -9.81])  # Assuming Z-axis is vertical downward
    return accel_global - gravity

# Main processing pipeline
def process_imu_data(file_path, freq=100):
    """
    Process IMU data to calculate vertical acceleration.
    file_path: Path to the CSV file
    freq: Sampling frequency of the IMU data (Hz)
    """
    # Load data
    data = load_imu_data(file_path)

    # Extract IMU readings
    accel = data[["accel_x", "accel_y", "accel_z"]].to_numpy()
    gyro = data[["gyro_x", "gyro_y", "gyro_z"]].to_numpy()

    # Initialize Madgwick filter
    madgwick = Madgwick(frequency=freq)

    # Initialize quaternion (identity quaternion for no rotation)
    q = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]

    # Process data step by step
    vertical_acceleration = []
    for i in range(len(accel)):
        # Update Madgwick filter with current gyro, accelerometer, and initial quaternion
        q = madgwick.updateIMU(q, gyr=gyro[i], acc=accel[i])

        # Transform accelerometer to global frame
        accel_global = transform_to_global(accel[i], q)

        # Subtract gravity to isolate linear acceleration
        accel_linear = subtract_gravity(accel_global)

        # Append vertical acceleration (Z-component)
        vertical_acceleration.append(accel_linear[2])

    # Add vertical acceleration to dataframe
    data["vertical_acceleration"] = vertical_acceleration

    # Save processed data
    output_file = file_path.replace(".csv", "_processed.csv")
    data.to_csv(output_file, index=False)
    print(f"Processed data saved to {output_file}")

# Example usage
if __name__ == "__main__":
    # Path to input CSV
    input_file = "testData.csv"  # Replace with your CSV file path
    
    # Process IMU data
    process_imu_data(input_file)



