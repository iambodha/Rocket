import pandas as pd
from scipy.io import savemat

# Load CSV data
data = pd.read_csv("testData.csv")

# Extract accelerometer and gyroscope data (no transpose)
imu_data = data[['AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ']].values

# Save as .mat file
savemat("testData.mat", {'imu_data': imu_data})
