import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Read CSV file
data = pd.read_csv('forceData.csv')

# Color palette: first 5 red tones, rest blue tones
high_contrast_colors = [
    '#FF0000',   # Bright Red
    '#FF4500',   # Orange Red
    '#DC143C',   # Crimson
    '#B22222',   # Fire Brick
    '#8B0000',   # Dark Red
    '#1E90FF',   # Dodger Blue
    '#4169E1',   # Royal Blue
    '#0000CD',   # Medium Blue
    '#0000FF',   # Blue
    '#00008B',   # Dark Blue
    '#191970',   # Midnight Blue
    '#4682B4'    # Steel Blue
]

# Create the scatter plot
plt.figure(figsize=(12, 8), facecolor='white')
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['axes.facecolor'] = 'white'

for i, piece_num in enumerate(data['Piece-Number'].unique()):
    subset = data[data['Piece-Number'] == piece_num]
    plt.scatter(subset['Distance'], subset['Force'], 
                label=f'Piece {piece_num}', 
                color=high_contrast_colors[i % len(high_contrast_colors)],
                edgecolors='black',
                linewidth=1,
                s=100,
                alpha=0.85)

plt.xlabel('Distance', fontweight='bold')
plt.ylabel('Force', fontweight='bold')
plt.title('Force vs Distance Scatter Plot', fontweight='bold')
plt.legend(frameon=True, facecolor='white', edgecolor='black')
plt.grid(True, linestyle='--', color='lightgray')

plt.tight_layout()
plt.show()