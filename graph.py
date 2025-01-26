import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

# Data
pressure = [6, 6, 6, 6, 6, 6, 6,
            5, 5, 5, 5, 5, 5, 5,
            4, 4, 4, 4, 4, 4, 4,
            3, 3, 3, 3, 3, 3, 3,
            2, 2, 2, 2, 2, 2, 2]

water_ratio = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
               0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
               0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
               0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,
               0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]

accelerationExp = [12.213, 15.842, 16.393, 11.92, 8.761, 5.237, 2.426,
                   8.398, 11.093, 13.005, 9.563, 6.602, 3.87, 1.402,
                   4.391, 7.074, 10.064, 6.938, 4.612, 2.248, 0.141,
                   2.843, 4.156, 7.406, 4.24, 2.982, 0.873, 0.38,
                   1.098, 1.285, 4.987, 1.814, 0.289, 0, 0]

accelerationSim = [24.578, 20.561, 16.612, 13.018, 9.691, 6.361, 3.454,
                   20.391, 16.928, 13.608, 10.555, 7.647, 4.897, 2.359,
                   16.16, 13.32, 10.526, 7.917, 5.606, 3.336, 1.196,
                   11.849, 9.583, 7.407, 5.396, 3.5, 1.704, 1.44,
                   7.479, 5.824, 4.249, 2.768, 1.326, 0.849, 0.627]

# Create a regular grid to interpolate the data
pressure_grid = np.linspace(min(pressure), max(pressure), 100)
water_ratio_grid = np.linspace(min(water_ratio), max(water_ratio), 100)
X, Y = np.meshgrid(pressure_grid, water_ratio_grid)

# Interpolate acceleration values on the regular grid
Z_exp = griddata((pressure, water_ratio), accelerationExp, (X, Y), method='cubic')
Z_sim = griddata((pressure, water_ratio), accelerationSim, (X, Y), method='cubic')

# Find the global min and max across both experimental and simulated data
vmin = min(np.min(Z_sim), np.min(Z_exp))
vmax = max(np.max(Z_sim), np.max(Z_exp))

# Plot the Experimental Contour in a new window
plt.figure(figsize=(10, 8))  # Create a new figure for the experimental plot
contour_exp = plt.contourf(X, Y, Z_exp, levels=15, cmap='viridis', vmin=vmin, vmax=vmax)
plt.colorbar(contour_exp, label='Acceleration (g)')
plt.xlabel('Pressure (Bar)', fontsize=14)
plt.ylabel('Water Ratio', fontsize=14)
plt.title('Experimental Acceleration Contour', fontsize=16)
plt.grid(True, linestyle='--', alpha=0.7)
plt.show()  # Show the plot in a separate window

# Plot the Simulated Contour in a new window
plt.figure(figsize=(10, 8))  # Create a new figure for the simulated plot
contour_sim = plt.contourf(X, Y, Z_sim, levels=15, cmap='viridis', vmin=vmin, vmax=vmax)
plt.colorbar(contour_sim, label='Acceleration (g)')
plt.xlabel('Pressure (Bar)', fontsize=14)
plt.ylabel('Water Ratio', fontsize=14)
plt.title('Simulated Acceleration Contour', fontsize=16)
plt.grid(True, linestyle='--', alpha=0.7)
plt.show()  # Show the plot in a separate window
