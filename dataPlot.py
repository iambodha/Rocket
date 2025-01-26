import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the CSV file into a pandas DataFrame
data = pd.read_csv('flightData.csv')

# Filter the data for the time range 55k to 65k ms
data_filtered = data[(data['Time(ms)'] >= 55000) & (data['Time(ms)'] <= 65000)]

# Convert acceleration values from g to m/s² (1 g = 9.81 m/s²)
g_to_m_s2 = 9.81
data_filtered['AccelX(m/s²)'] = data_filtered['AccelX(g)'] * g_to_m_s2
data_filtered['AccelY(m/s²)'] = data_filtered['AccelY(g)'] * g_to_m_s2
data_filtered['AccelZ(m/s²)'] = (data_filtered['AccelZ(g)'] - 1) * g_to_m_s2  # Subtract 1g for gravity compensation

# Calculate altitude from pressure using the barometric formula
P0 = 101325  # Standard atmospheric pressure at sea level (Pa)
data_filtered['Altitude(m)'] = 44330 * (1 - (data_filtered['Pressure(Pa)'] / P0) ** (1 / 5.255))

# Plot Acceleration (AccelX, AccelY, AccelZ)
plt.figure(figsize=(12, 8))

plt.subplot(2, 1, 1)
plt.plot(data_filtered['Time(ms)'], data_filtered['AccelX(m/s²)'], label='AccelX')
plt.plot(data_filtered['Time(ms)'], data_filtered['AccelY(m/s²)'], label='AccelY')
plt.plot(data_filtered['Time(ms)'], data_filtered['AccelZ(m/s²)'], label='AccelZ')
plt.title('Acceleration in X, Y, and Z axes (in m/s²)')
plt.xlabel('Time (ms)')
plt.ylabel('Acceleration (m/s²)')
plt.legend(loc='upper right')

# Plot Altitude
plt.subplot(2, 1, 2)
plt.plot(data_filtered['Time(ms)'], data_filtered['Altitude(m)'], label='Altitude', color='green')
plt.title('Estimated Altitude')
plt.xlabel('Time (ms)')
plt.ylabel('Altitude (m)')
plt.legend(loc='upper right')

plt.tight_layout()
plt.show()
