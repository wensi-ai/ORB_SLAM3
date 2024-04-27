import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from scipy.spatial.transform import Rotation as R
import csv

# Parse arguments
parser = argparse.ArgumentParser(description='Visualize camera trajectory')
parser.add_argument(
    '-i', '--input', 
    type=str, 
    help='Path to camera trajectory file'
)
args = parser.parse_args()

# parse camera trajectory txt file
positions, orientations = [], []
with open(args.input, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        positions.append([float(row['x']), float(row['y']), float(row['z'])])
        orientations.append([float(row['q_x']), float(row['q_y']), float(row['q_z']), float(row['q_w'])])
positions = np.array(positions) 
orientations = R.from_quat(orientations)
num_frames = len(positions) 

# Create figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 2])
ax.set_ylim([-1, 2])
ax.set_zlim([-1, 1])

# Initialize Line3D objects for position and orientation
line_pos, = ax.plot([], [], [], 'y-')
line_r, = ax.plot([], [], [], 'r-')
line_p, = ax.plot([], [], [], 'g-')
line_y, = ax.plot([], [], [], 'b-')

# Function to initialize the plot
def init():
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)
    ax.grid(True)
    return line_pos, line_r, line_p, line_y

# Function to update the plot for each frame of animation
def update(frame):
    # Update camera position
    line_pos.set_data(positions[:frame+1,0], positions[:frame+1,1])
    line_pos.set_3d_properties(positions[:frame+1,2])
    
    # Update camera orientation
    end_point_r = positions[frame] + orientations[frame].apply([1, 0, 0]) * 0.25  # Scale orientation vector for visualization
    line_r.set_data([positions[frame,0], end_point_r[0]], [positions[frame,1], end_point_r[1]])
    line_r.set_3d_properties([positions[frame,2], end_point_r[2]])
    end_point_p = positions[frame] + orientations[frame].apply([0, 1, 0]) * 0.25  
    line_p.set_data([positions[frame,0], end_point_p[0]], [positions[frame,1], end_point_p[1]])
    line_p.set_3d_properties([positions[frame,2], end_point_p[2]])
    end_point_y = positions[frame] + orientations[frame].apply([0, 0, 1]) * 0.25  
    line_y.set_data([positions[frame,0], end_point_y[0]], [positions[frame,1], end_point_y[1]])
    line_y.set_3d_properties([positions[frame,2], end_point_y[2]])

    return line_pos, line_r, line_p, line_y

# Create animation
ani = animation.FuncAnimation(fig, update, frames=np.arange(0, num_frames), init_func=init, blit=True, interval=20)

# Show the plot
plt.show()
