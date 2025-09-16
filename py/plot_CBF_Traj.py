import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
from matplotlib import animation
import mpl_toolkits.mplot3d.art3d as art3d

def savefig(plt, path, column=1):
    """Save figure with proper dimensions and format"""
    cm_to_inc = 1 / 2.54  # Centimeter to inch conversion (1 inch = 2.54 cm)
    gcf = plt.gcf()  # Get current figure
    
    if column == 1:
        gcf.set_size_inches(8 * cm_to_inc, 5.33 * cm_to_inc)
    else:
        gcf.set_size_inches(16.33 * cm_to_inc, 10.33 * cm_to_inc)
    
    plt.savefig(path, dpi=600, bbox_inches='tight', format='svg')

def square_elements(arr):
    """Square each element in the array"""
    result = []
    for num in arr:
        result.append(num ** 2)
    return result

# Font and style settings
plt.rcParams['font.sans-serif'] = ['Times New Roman']
plt.rcParams['font.weight'] = 'light'
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['axes.titlesize'] = 8
plt.rcParams['axes.labelsize'] = 6
plt.rcParams['xtick.labelsize'] = 7
plt.rcParams['ytick.labelsize'] = 7
plt.rcParams['legend.fontsize'] = 6
plt.rcParams['axes.linewidth'] = 1
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'

def limit(x):
    """Limit values to the range [-π, π]"""
    for iter in range(len(x)):
        if x[iter] > 3.14159:
            x[iter] = 3.14159 * 2 - x[iter]
        if x[iter] < -3.14158:
            x[iter] = 3.14159 * 2 + x[iter]
    return x

# Read data from file
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_0P3.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_1P4.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLG_ALPHA_1P4_BETA_0P0.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLG_ALPHA_1P4_BETA_0P3.txt'
file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_1P4_BETA_1P3.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log.txt'
data = np.loadtxt(file)

# Set start and end lines
start_line = 0
end_line = 700
if end_line is None or end_line > len(data):
    end_line = len(data)

selected_data = data[start_line:end_line]

# Parse data columns
px_mpc = selected_data[:, 0]  # Robot X position
py_mpc = selected_data[:, 1]  # Robot Y position
pz_mpc = selected_data[:, 2]  # Robot Z position
rx = selected_data[:, 3]      # Robot X rotation
ry = selected_data[:, 4]      # Robot Y rotation
rz = selected_data[:, 5]      # Robot Z rotation
vx = selected_data[:, 6]      # Robot X velocity
vy = selected_data[:, 7]      # Robot Y velocity
vz = selected_data[:, 8]      # Robot Z velocity
gx = selected_data[:, 9]      # Gyro X
gy = selected_data[:, 10]     # Gyro Y
gz = selected_data[:, 11]     # Gyro Z
in1 = selected_data[:, 12]    # Input 1 (Thrust)
in2 = selected_data[:, 13]    # Input 2 (Angular velocity X)
in3 = selected_data[:, 14]    # Input 3 (Angular velocity Y)
in4 = selected_data[:, 15]    # Input 4 (Angular velocity Z)

px_ref = selected_data[:, 16]  # Reference X position
py_ref = selected_data[:, 17]  # Reference Y position
pz_ref = selected_data[:, 18]  # Reference Z position
rx_ref = selected_data[:, 19]  # Reference X rotation
ry_ref = selected_data[:, 20]  # Reference Y rotation
rz_ref = selected_data[:, 21]  # Reference Z rotation
vx_ref = selected_data[:, 22]  # Reference X velocity
vy_ref = selected_data[:, 23]  # Reference Y velocity
vz_ref = selected_data[:, 24]  # Reference Z velocity
gx_ref = selected_data[:, 25]  # Reference gyro X
gy_ref = selected_data[:, 26]  # Reference gyro Y
gz_ref = selected_data[:, 27]  # Reference gyro Z
time_cost = selected_data[:, 32]  # Optimization time cost

# Obstacle data
obs_px = selected_data[:, 33]  # Obstacle X position
obs_py = selected_data[:, 34]  # Obstacle Y position
obs_pz = selected_data[:, 35]  # Obstacle Z position
obs_vx = selected_data[:, 36]  # Obstacle X velocity
obs_vy = selected_data[:, 37]  # Obstacle Y velocity
obs_vz = selected_data[:, 38]  # Obstacle Z velocity

# Create time axis
time_steps = np.arange(len(px_mpc))

print(f"Plotting data range: Line {start_line} to {end_line}")
print(f"Total of {len(px_mpc)} data points")

def create_cylinder(ax, x, y, z, radius, height, color='red', alpha=0.3):
    """Create a cylinder at the specified position"""
    # Create cylinder mesh
    z_cyl = np.linspace(z, z + height, 50)
    theta = np.linspace(0, 2 * np.pi, 50)
    theta_grid, z_grid = np.meshgrid(theta, z_cyl)
    x_grid = radius * np.cos(theta_grid) + x
    y_grid = radius * np.sin(theta_grid) + y
    
    # Plot cylinder surface
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha, linewidth=0)
    
    # Plot top and bottom circles
    theta_circle = np.linspace(0, 2 * np.pi, 100)
    x_circle = radius * np.cos(theta_circle) + x
    y_circle = radius * np.sin(theta_circle) + y
    
    # Bottom disk
    ax.plot(x_circle, y_circle, z * np.ones_like(x_circle), color=color, alpha=alpha)
    # Top disk
    ax.plot(x_circle, y_circle, (z + height) * np.ones_like(x_circle), color=color, alpha=alpha)
    
    return ax

# Figure 1: 3D trajectories with cylindrical obstacle
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Cylinder parameters
cylinder_radius = 0.10  # Cylinder radius 0.10 m
cylinder_height = 1.5   # Cylinder height 1.5 m

# Plot robot trajectory (color represents time)
sc1 = ax.scatter(px_mpc, py_mpc, pz_mpc, c=time_steps, cmap='viridis', s=0.5, label='Quad', alpha=0.8)

# Plot Cylinder
# create_cylinder(ax, obs_px[0], obs_py[0], 0, cylinder_radius, cylinder_height, 'red', 0.30)

# Plot obstacle trajectory points (color represents time)
sc2 = ax.scatter(obs_px, obs_py, obs_pz, c=time_steps, cmap='plasma', s=0.3, label='Cylinder', alpha=0.6)

# Plot reference trajectory
ax.plot(px_ref, py_ref, pz_ref, 'r--', linewidth = 1, label='Ref', alpha=0.7)

# Set axis limits
ax.set_xlim(min(np.min(px_mpc), np.min(obs_px)) - 0.5, max(np.max(px_mpc), np.max(obs_px)) + 0.5)
# ax.set_ylim(-0.20, 1.0)

# Axis labels
ax.set_xlabel('X (m)', fontsize=8, fontname='Times New Roman')
ax.set_ylabel('Y (m)', fontsize=8, fontname='Times New Roman')
ax.set_zlabel('Z (m)', fontsize=8, fontname='Times New Roman')

# Add colorbar
cbar = fig.colorbar(sc1, ax=ax, shrink=0.8)
cbar.set_label('Time Step', fontsize=8, fontname='Times New Roman')

ax.legend(prop={'family': 'Times New Roman', 'size': 8})
plt.tight_layout()
savefig(plt, "3d_trajectories_with_cylinder.svg", column=1)

# Figure 2: Velocity and other parameter curves over time
fig, axes = plt.subplots(4, 2, figsize=(12, 16))

# Target velocity - XYZ components
axes[0, 0].plot(time_steps, vx_ref, 'r-', linewidth=1, label='X')
axes[0, 0].plot(time_steps, vy_ref, 'g-', linewidth=1, label='Y')
axes[0, 0].plot(time_steps, vz_ref, 'b-', linewidth=1, label='Z')
axes[0, 0].set_ylabel('Velocity (m/s)', fontsize=8, fontname='Times New Roman')
axes[0, 0].set_title('Reference Velocity (XYZ)', fontsize=8, fontname='Times New Roman')
axes[0, 0].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[0, 0].grid(True, alpha=0.3)

# Obstacle velocity - XYZ components
axes[0, 1].plot(time_steps, obs_vx, 'r-', linewidth=1, label='X')
axes[0, 1].plot(time_steps, obs_vy, 'g-', linewidth=1, label='Y')
axes[0, 1].plot(time_steps, obs_vz, 'b-', linewidth=1, label='Z')
axes[0, 1].set_ylabel('Velocity (m/s)', fontsize=8, fontname='Times New Roman')
axes[0, 1].set_title('Obstacle Velocity (XYZ)', fontsize=8, fontname='Times New Roman')
axes[0, 1].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[0, 1].grid(True, alpha=0.3)

# 在axes[1, 0]绘制障碍物平面轨迹和Quad轨迹
axes[1, 0].plot(obs_px, obs_py, 'r-', linewidth=1, label='Obs', alpha=0.7)
axes[1, 0].plot(px_mpc, py_mpc, 'b-', linewidth=1, label='Quad', alpha=0.7)
axes[1, 0].set_xlabel('X (m)', fontsize=8, fontname='Times New Roman')
axes[1, 0].set_ylabel('Y (m)', fontsize=8, fontname='Times New Roman')
axes[1, 0].set_title('XY Plane Trajectories', fontsize=8, fontname='Times New Roman')
axes[1, 0].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[1, 0].grid(True, alpha=0.3)
axes[1, 0].set_aspect('equal', adjustable='datalim')  # 保持纵横比相等

# Robot velocity - XYZ components
axes[1, 1].plot(time_steps, vx, 'r-', linewidth=1, label='Robot X')
axes[1, 1].plot(time_steps, vy, 'g-', linewidth=1, label='Robot Y')
axes[1, 1].plot(time_steps, vz, 'b-', linewidth=1, label='Robot Z')
axes[1, 1].set_xlabel('Time Step', fontsize=8, fontname='Times New Roman')  # 添加x轴标签
axes[1, 1].set_ylabel('Velocity (m/s)', fontsize=8, fontname='Times New Roman')
axes[1, 1].set_title('Robot Velocity (XYZ)', fontsize=8, fontname='Times New Roman')
axes[1, 1].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[1, 1].grid(True, alpha=0.3)

# Angular velocity command - XYZ components
axes[2, 0].plot(time_steps, in2, 'r-', linewidth=1, label='ωX')
axes[2, 0].plot(time_steps, in3, 'g-', linewidth=1, label='ωY')
axes[2, 0].plot(time_steps, in4, 'b-', linewidth=1, label='ωZ')
axes[2, 0].set_ylabel('Angular Velocity (rad/s)', fontsize=8, fontname='Times New Roman')
axes[2, 0].set_title('Angular Velocity Command (XYZ)', fontsize=8, fontname='Times New Roman')
axes[2, 0].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[2, 0].grid(True, alpha=0.3)

# Thrust command
axes[2, 1].plot(time_steps, in1, 'r-', linewidth=1, label='Thrust')
axes[2, 1].set_ylabel('Thrust (N)', fontsize=8, fontname='Times New Roman')
axes[2, 1].set_title('Thrust Command', fontsize=8, fontname='Times New Roman')
axes[2, 1].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[2, 1].grid(True, alpha=0.3)

# Robot rotation - XYZ components
axes[3, 0].plot(time_steps, rx, 'r-', linewidth=1, label='Rx')
axes[3, 0].plot(time_steps, ry, 'g-', linewidth=1, label='Ry')
axes[3, 0].plot(time_steps, rz, 'b-', linewidth=1, label='Rz')
axes[3, 0].set_xlabel('Time Step', fontsize=8, fontname='Times New Roman')  # 添加x轴标签
axes[3, 0].set_ylabel('Rotation (rad)', fontsize=8, fontname='Times New Roman')
axes[3, 0].set_title('Rotation (XYZ)', fontsize=8, fontname='Times New Roman')
axes[3, 0].legend(prop={'family': 'Times New Roman', 'size': 7})
axes[3, 0].grid(True, alpha=0.3)

# Optimization cost
axes[3, 1].plot(time_steps, time_cost, 'k-', linewidth=1)
axes[3, 1].set_xlabel('Time Step', fontsize=8, fontname='Times New Roman')
axes[3, 1].set_ylabel('Cost (s)', fontsize=8, fontname='Times New Roman')
axes[3, 1].set_title('Optimization Cost', fontsize=8, fontname='Times New Roman')
axes[3, 1].grid(True, alpha=0.3)

# 调整子图间距，防止标题和坐标轴重叠
plt.subplots_adjust(hspace=0.4, wspace=0.8)

plt.tight_layout()
savefig(plt, "velocity_curves.svg", column=2)

# Figure 3: Distance between robot and obstacle
plt.figure()
distances = []
distances_b2b = []
UAV_SIZE = 0.20
OBS_RADIUS = 0.10

for i in range(len(px_mpc)):
    dist = np.sqrt((px_mpc[i] - obs_px[i])**2 + 
                  (py_mpc[i] - obs_py[i])**2)
    distances.append(dist)
    distances_b2b.append(dist - OBS_RADIUS - UAV_SIZE/2)  # Subtract robot and obstacle radii

plt.plot(time_steps, distances_b2b, 'b-', linewidth=2, label='Distance')
plt.axhline(y=0.05, color='orange', linestyle=':', linewidth=1, label='Safety Margin (0.05m)')

plt.xlabel('Time Step', fontname='Times New Roman')
plt.ylabel('Distance (m)', fontname='Times New Roman')
plt.title('Doundary Distance between Quadrotor and Obstacle', fontname='Times New Roman')
plt.grid(True, alpha=0.3)
plt.legend(prop={'family': 'Times New Roman', 'size': 8})
plt.tight_layout()
# plt.xlim(200, 500)
# plt.ylim(0, 0.25)
savefig(plt, "distance_to_obstacle.svg", column=1)

plt.show()