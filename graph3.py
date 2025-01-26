import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib.animation import FuncAnimation

# Read data
df = pd.read_csv('flightData.csv')

# Convert time to seconds from start
df['Time(s)'] = (df['Time(ms)'] - df['Time(ms)'].iloc[0]) / 1000

# Convert angular velocities to RPS
df['Roll (RPS)'] = abs(df['GyroX(deg/s)'] / 360)
df['Pitch (RPS)'] = abs(df['GyroY(deg/s)'] / 360)
df['Yaw (RPS)'] = abs(df['GyroZ(deg/s)'] / 360)

# Create figure for 3D plot
fig = plt.figure(figsize=(10, 8))
ax1 = fig.add_subplot(111, projection='3d')

def create_rocket():
    # Body
    theta = np.linspace(0, 2 * np.pi, 20)
    z = np.linspace(0, 3, 20)
    theta, z = np.meshgrid(theta, z)
    radius = 0.3 * np.ones_like(z)
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    
    # Nose cone
    nose_z = np.linspace(3, 4, 10)
    nose_radius = np.linspace(0.3, 0, 10)
    nose_theta = np.linspace(0, 2 * np.pi, 20)
    nose_theta, nose_z = np.meshgrid(nose_theta, nose_z)
    nose_x = nose_radius[:, np.newaxis] * np.cos(nose_theta)
    nose_y = nose_radius[:, np.newaxis] * np.sin(nose_theta)
    
    # Arrow points (arrow on the side of the rocket)
    arrow_base = np.array([0.3, 0, 1.5])  # Base of arrow
    arrow_tip = np.array([0.8, 0, 1.5])   # Tip of arrow
    arrow_head1 = np.array([0.6, 0, 1.7]) # Upper head point
    arrow_head2 = np.array([0.6, 0, 1.3]) # Lower head point
    
    return x, y, z, nose_x, nose_y, nose_z, arrow_base, arrow_tip, arrow_head1, arrow_head2

# Initialize rocket
rocket_x, rocket_y, rocket_z, nose_x, nose_y, nose_z, arrow_base, arrow_tip, arrow_head1, arrow_head2 = create_rocket()

def update(frame):
    ax1.cla()
    
    # Calculate cumulative rotation at this frame
    rot_x = np.cumsum(df['GyroX(deg/s)'].values)
    rot_y = np.cumsum(df['GyroY(deg/s)'].values)
    rot_z = np.cumsum(df['GyroZ(deg/s)'].values)
    
    # Get current rotation
    idx = int(frame % len(df))
    current_rot_x = (rot_x[idx] * np.pi / 180)*0
    current_rot_y = (rot_y[idx] * np.pi / 180)*0
    current_rot_z = rot_z[idx] * np.pi / 180
    
    # Create rotation matrices
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(current_rot_x), -np.sin(current_rot_x)],
                   [0, np.sin(current_rot_x), np.cos(current_rot_x)]])
    
    Ry = np.array([[np.cos(current_rot_y), 0, np.sin(current_rot_y)],
                   [0, 1, 0],
                   [-np.sin(current_rot_y), 0, np.cos(current_rot_y)]])
    
    Rz = np.array([[np.cos(current_rot_z), -np.sin(current_rot_z), 0],
                   [np.sin(current_rot_z), np.cos(current_rot_z), 0],
                   [0, 0, 1]])
    
    R = Rz @ Ry @ Rx
    
    # Plot rocket body
    for i in range(len(rocket_z)):
        points = np.vstack((rocket_x[i], rocket_y[i], rocket_z[i]))
        rotated_points = R @ points
        ax1.plot(rotated_points[0], rotated_points[1], rotated_points[2], 'b-', alpha=0.5)
    
    # Plot nose cone
    for i in range(len(nose_z)):
        points = np.vstack((nose_x[i], nose_y[i], nose_z[i]))
        rotated_points = R @ points
        ax1.plot(rotated_points[0], rotated_points[1], rotated_points[2], 'r-', alpha=0.5)
    
    # Plot arrow
    arrow_base_rotated = R @ arrow_base
    arrow_tip_rotated = R @ arrow_tip
    ax1.plot([arrow_base_rotated[0], arrow_tip_rotated[0]], 
             [arrow_base_rotated[1], arrow_tip_rotated[1]], 
             [arrow_base_rotated[2], arrow_tip_rotated[2]], 
             'g-', linewidth=2)
    
    arrow_head1_rotated = R @ arrow_head1
    arrow_head2_rotated = R @ arrow_head2
    ax1.plot([arrow_tip_rotated[0], arrow_head1_rotated[0]], 
             [arrow_tip_rotated[1], arrow_head1_rotated[1]], 
             [arrow_tip_rotated[2], arrow_head1_rotated[2]], 
             'g-', linewidth=2)
    ax1.plot([arrow_tip_rotated[0], arrow_head2_rotated[0]], 
             [arrow_tip_rotated[1], arrow_head2_rotated[1]], 
             [arrow_tip_rotated[2], arrow_head2_rotated[2]], 
             'g-', linewidth=2)
    
    ax1.set_xlim([-2, 2])
    ax1.set_ylim([-2, 2])
    ax1.set_zlim([0, 5])
    ax1.set_title('Rocket Orientation')
    ax1.set_box_aspect([1, 1, 1])

# Calculate real-time interval
flight_duration = 46570 - 36935  # in ms
num_frames = 397
real_time_interval = flight_duration / num_frames  # in ms

# Create animation with real-time speed
anim = FuncAnimation(fig, update, frames=num_frames, interval=real_time_interval)

# To save the animation, uncomment and modify one of these methods:
# Method 1: Save as MP4 (requires ffmpeg)
#anim.save('rocket_orientation.mp4', writer='ffmpeg', fps=30)

# Method 2: Save as GIF (slower, larger file)
anim.save('rocket_orientation.gif', writer='pillow')

plt.tight_layout()
plt.show()