import matplotlib.pyplot as plt
import pandas as pd

# Load the first dataset
df_experiment = pd.read_csv('experiment.csv')
time_experiment = df_experiment['time']
accel_experiment = df_experiment['total_accel']

# Load the second dataset
df_simulation = pd.read_csv('rocketDataSim.csv')
time_simulation = df_simulation['Time'] * 1000  # Convert seconds to milliseconds
accel_simulation = df_simulation['Acceleration']

# Define the cutoff time in milliseconds
cutoff_time = 5000

# Filter the data to only include times up to the cutoff
df_experiment_filtered = df_experiment[time_experiment <= cutoff_time]
time_experiment_filtered = df_experiment_filtered['time']
accel_experiment_filtered = df_experiment_filtered['total_accel']

df_simulation_filtered = df_simulation[time_simulation <= cutoff_time]
time_simulation_filtered = df_simulation_filtered['Time'] * 1000  # Ensure it's in milliseconds
accel_simulation_filtered = df_simulation_filtered['Acceleration']

# Plot the filtered data
plt.figure(figsize=(10, 6))
plt.plot(time_experiment_filtered, accel_experiment_filtered, label='Experiment Data', color='blue', marker='o')
plt.plot(time_simulation_filtered, accel_simulation_filtered, label='Simulation Data', color='red', linestyle='--')

# Add titles and labels
plt.title('Experimental vs. Simulated Acceleration for 0.3 Water Ratio and 6B Flight')
plt.xlabel('Time (ms)', fontsize=14)
plt.ylabel('Acceleration (g)', fontsize=14)
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
