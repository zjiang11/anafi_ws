import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# Directory and file path
data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data','move2ref','newton_euler_mpc')
file_path = os.path.join(data_dir, 'drone_data.csv')

# Load the CSV data
df = pd.read_csv(file_path)

# Ensure all columns are in numeric format (converts non-numeric entries to NaN)
for col in ['Timestamp', 'Reference X', 'Current X', 'Input X',
            'Reference Y', 'Current Y', 'Input Y',
            'Reference Z', 'Current Z', 'Input Z',
            'Reference Yaw', 'Current Yaw', 'Input Yaw']:
    df[col] = pd.to_numeric(df[col], errors='coerce')

# Drop any rows with NaN values
df = df.dropna()

# Convert to numpy arrays to ensure 1D format
timestamp = np.array(df['Timestamp'])
ref_x = np.array(df['Reference X'])
curr_x = np.array(df['Current X'])
curr_x_input = np.array(df['Input X'])
ref_y = np.array(df['Reference Y'])
curr_y = np.array(df['Current Y'])
curr_y_input = np.array(df['Input Y'])
ref_z = np.array(df['Reference Z'])
curr_z = np.array(df['Current Z'])
curr_z_input = np.array(df['Input Z'])
ref_yaw = np.array(df['Reference Yaw'])
curr_yaw = np.array(df['Current Yaw'])
curr_yaw_input = np.array(df['Input Yaw'])

# Create a 4x1 figure
fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot X-coordinates with secondary y-axis for X_output
ax1 = axs[0]
ax2 = ax1.twinx()
ax1.plot(timestamp, ref_x, label='Reference X', color='blue')
ax1.plot(timestamp, curr_x, label='Position X', color='green')
ax2.plot(timestamp, curr_x_input, label='Input X', color='red', linestyle='--')
ax1.set_ylabel('X-axis Position')
ax2.set_ylabel('X-axis Input')
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')
ax1.set_title('Drone X, Y, Z, Yaw vs. Time')

# Plot Y-coordinates with secondary y-axis for Y_output
ax1 = axs[1]
ax2 = ax1.twinx()
ax1.plot(timestamp, ref_y, label='Reference Y', color='blue')
ax1.plot(timestamp, curr_y, label='Position Y', color='green')
ax2.plot(timestamp, curr_y_input, label='Input Y', color='red', linestyle='--')
ax1.set_ylabel('Y-axis Position')
ax2.set_ylabel('Y-axis Input')
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Plot Z-coordinates with secondary y-axis for Z_output
ax1 = axs[2]
ax2 = ax1.twinx()
ax1.plot(timestamp, ref_z, label='Reference Z', color='blue')
ax1.plot(timestamp, curr_z, label='Position Z', color='green')
ax2.plot(timestamp, curr_z_input, label='Input Z', color='red', linestyle='--')
ax1.set_ylabel('Z-axis Position')
ax2.set_ylabel('Z-axis Input')
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Plot Yaw with secondary y-axis for Yaw_output
ax1 = axs[3]
ax2 = ax1.twinx()
ax1.plot(timestamp, ref_yaw, label='Reference Yaw', color='blue')
ax1.plot(timestamp, curr_yaw, label='Position Yaw', color='green')
ax2.plot(timestamp, curr_yaw_input, label='Input Yaw', color='red', linestyle='--')
ax1.set_ylabel('Yaw Position')
ax2.set_ylabel('Yaw Input')
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')

# Set shared x-axis label for all subplots
axs[3].set_xlabel('Timestamp')

# Improve layout
plt.tight_layout()

# Save the figure
plt.savefig(os.path.join(data_dir, 'drone_2d_trajectory.png'))
plt.show()



fig_3d = plt.figure(figsize=(10, 8))
ax_3d = fig_3d.add_subplot(111, projection='3d')

# Plot reference trajectory
ax_3d.plot(ref_x, ref_y, ref_z, label='Reference Trajectory', color='blue', linewidth=2)

# Plot current trajectory
ax_3d.plot(curr_x, curr_y, curr_z, label='Current Trajectory', color='green', linestyle='--', linewidth=2)

# Labels and legend
ax_3d.set_xlabel('X Position')
ax_3d.set_ylabel('Y Position')
ax_3d.set_zlabel('Z Position')
ax_3d.set_title('3D Trajectory: Reference vs. Current')
ax_3d.legend()

# Save and show
plt.tight_layout()
plt.savefig(os.path.join(data_dir, 'drone_3d_trajectory.png'))
plt.show()