import pandas as pd
import matplotlib.pyplot as plt
import os

# Load CSV file
data_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'track_parrot_drone')
save_dir = os.path.join(os.path.expanduser("~"), 'anafi_ws', 'data', 'track_parrot_drone','figures')
os.makedirs(save_dir, exist_ok=True)
csv_file = os.path.join(data_dir, 'drone_data.csv')
df = pd.read_csv(csv_file)

fig = plt.figure(figsize=(10,6))

ax3d = fig.add_subplot(projection = '3d')
ax3d.view_init(elev=20, azim=150)
 
ax3d.set_title("Real-Time 3D Trajectory")
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")
ax3d.set_xlim([-2, 2])
ax3d.set_ylim([-2, 2])
ax3d.set_zlim([0, 3])

time_stamp = df["Timestamp"].to_numpy()[:500]
parrot_x = df["Parrot X"].to_numpy()[:500]
parrot_y = df["Parrot Y"].to_numpy()[:500]
parrot_z = df["Parrot Z"].to_numpy()[:500]
anafi_x = df["Anafi X"].to_numpy()[:500]
anafi_y = df["Anafi Y"].to_numpy()[:500]
anafi_z = df["Anafi Z"].to_numpy()[:500]

# ax3d.set_title("Real-Time 3D Trajectory")
ax3d.set_xlabel("X Position (m)")
ax3d.set_ylabel("Y Position (m)")
ax3d.set_zlabel("Z Position (m)")
ax3d.set_xlim([-2, 2])
ax3d.set_ylim([-2, 2])
ax3d.set_zlim([0, 3])

ax3d.plot(parrot_x, parrot_y, parrot_z, 'ro-', label="target drone")
ax3d.plot(anafi_x, anafi_y, anafi_z, 'bo-', label="pursuing drone")

ax3d.legend()

save_path = os.path.join(save_dir, "3d_trajectory_plot.png")
plt.savefig(save_path, dpi=300, bbox_inches="tight")
plt.show()




fig_x = plt.figure(figsize=(10, 6))
ax_x = fig_x.add_subplot(1, 1, 1)
ax_x.set_ylabel("X Position (m)")
ax_x.set_xlabel("Time (s)")
ax_x.set_xlim([0, 20])
ax_x.set_ylim([-3, 4.5])
ax_x.plot(time_stamp, parrot_x, 'r-', label="target drone")
ax_x.plot(time_stamp, anafi_x, 'r--', label="pursuing drone")
ax_x.legend(loc="upper right", fontsize=10, frameon=True)
# plt.title("X Position vs Time")
plt.subplots_adjust(hspace=0.8) 
save_path = os.path.join(save_dir, "2d_trajectory_plot_x.png")
plt.savefig(save_path, dpi=300, bbox_inches="tight")
plt.show()




fig_y = plt.figure(figsize=(10, 6))
ax_y = fig_y.add_subplot(1, 1, 1)
ax_y.set_ylabel("Y Position (m)")
ax_y.set_xlabel("Time (s)")
ax_y.set_xlim([0, 20])
ax_y.set_ylim([-2, 2.5])
ax_y.plot(time_stamp, parrot_y, 'g-', label="target drone")
ax_y.plot(time_stamp, anafi_y, 'g--', label="pursuing drone")
ax_y.legend(loc="upper right", fontsize=10, frameon=True)
# plt.title("Y Position vs Time")
plt.subplots_adjust(hspace=0.8) 
save_path = os.path.join(save_dir, "2d_trajectory_plot_y.png")
plt.savefig(save_path, dpi=300, bbox_inches="tight")
plt.show()




fig_z = plt.figure(figsize=(10, 6))
ax_z = fig_z.add_subplot(1, 1, 1)
ax_z.set_ylabel("Z Position (m)")
ax_z.set_xlabel("Time (s)")
ax_z.set_xlim([0, 20])
ax_z.set_ylim([0.5, 2.5])
ax_z.plot(time_stamp, parrot_z, 'b-', label="target drone")
ax_z.plot(time_stamp, anafi_z, 'b--', label="pursuing drone")
ax_z.legend(loc="upper right", fontsize=10, frameon=True)
# plt.title("Z Position vs Time")
plt.subplots_adjust(hspace=0.8) 
save_path = os.path.join(save_dir, "2d_trajectory_plot_z.png")
plt.savefig(save_path, dpi=300, bbox_inches="tight")
plt.show()
