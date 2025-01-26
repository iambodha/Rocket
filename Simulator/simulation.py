import matplotlib.pyplot as plt
import numpy as np

# Parameters for bottle sections (in meters)
h_main = 0.15  # 15 cm to m
d_main = 0.08  # 8 cm to m

h_cone1 = 0.03  # 3 cm to m
d_cone1_start = d_main  # 8 cm to m
d_cone1_end = 0.074  # 7.4 cm to m

h_cone2 = 0.07  # 7 cm to m
d_cone2_start = d_cone1_end  # 7.4 cm to m
d_cone2_end = 0.019  # 2.8 cm to m

h_nozzle = 0.02  # 2 cm to m
d_nozzle = 0.019  # 1.9 cm to m

# Function to draw 2D bottle section (flipped vertically)
def draw_section_2d(y_start, height, d_start, d_end):
    # Coordinates for the left and right sides of the section
    y = [y_start, y_start - height]
    x_left = [-d_start / 2, -d_end / 2]
    x_right = [d_start / 2, d_end / 2]
    
    # Draw left and right sides of the section
    plt.plot(x_left, y, 'k', linewidth=2)
    plt.plot(x_right, y, 'k', linewidth=2)

# Create the plot
plt.figure(figsize=(6, 8))
plt.gca().set_aspect('equal', adjustable='box')

# Draw the bottle sections (upside down)
draw_section_2d(0, h_main, d_main, d_main)  # Main cylinder
draw_section_2d(-h_main, h_cone1, d_cone1_start, d_cone1_end)  # Cone 1
draw_section_2d(-(h_main + h_cone1), h_cone2, d_cone2_start, d_cone2_end)  # Cone 2
draw_section_2d(-(h_main + h_cone1 + h_cone2), h_nozzle, d_nozzle, d_nozzle)  # Nozzle

# Draw the bottom line (which is now the top due to flipping)
plt.plot([-d_main / 2, d_main / 2], [0, 0], 'k', linewidth=2)

# Set plot limits and labels
plt.xlim([-d_main / 2 * 1.2, d_main / 2 * 1.2])
plt.ylim([-(h_main + h_cone1 + h_cone2 + h_nozzle) * 1.1, 0])
plt.xlabel('Width (m)')
plt.ylabel('Height (m)')
plt.title('Upside Down Bottle Visualization')
plt.grid(True)

# Show the plot
plt.show()
