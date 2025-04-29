import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import sys

# Get the CSV filename from the command line argument
if len(sys.argv) != 2:
    print("Usage: python plot_path.py <csv_file>")
    sys.exit(1)

csv_file = sys.argv[1]

# Load the processed output
df = pd.read_csv(csv_file)

# Separate by section
obstacles = df[df['section'] == 'Obstacle']
start = df[df['section'] == 'Start']
goal = df[df['section'] == 'Goal']
path = df[df['section'] == 'Path']

# Setup plot
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot obstacles as transparent spheres
for _, row in obstacles.iterrows():
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = 0.05 * np.cos(u) * np.sin(v) + row['x']  # Approx obstacle radius as 5cm
    y = 0.05 * np.sin(u) * np.sin(v) + row['y']
    z = 0.05 * np.cos(v) + row['z']
    ax.plot_surface(x, y, z, color='red', alpha=0.3, edgecolor='none')

# Plot path
ax.plot(path['x'], path['y'], path['z'], label='Planned Path', color='blue', linewidth=2)

# Plot start points
ax.scatter(start['x'], start['y'], start['z'], color='green', s=50, label='Start Pose')

# Plot goal points
ax.scatter(goal['x'], goal['y'], goal['z'], color='orange', s=50, label='Goal Pose')

# Labels and title
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('RRT* Path in 3D Cartesian Space')

# Equal aspect ratio
ax.set_box_aspect([1,1,1])  # Force box to be cube-like

# Grid, legend
ax.grid(True)
ax.legend()

plt.show()
