import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math
from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d
from matplotlib import animation
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

def savefig(plt, path, column=1):
    cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
    gcf = plt.gcf()  # 获取当前图像
    if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
    else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
    plt.savefig(path, dpi=600, bbox_inches='tight', format='svg')

def square_elements(arr):
    result = []
    for num in arr:
        result.append(num ** 2)
    return result

# 字体调整
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
    for iter in range(len(x)):
        if x[iter] > 3.14159:
            x[iter] = 3.14159*2-x[iter]
        if x[iter] < -3.14158:
            x[iter] = 3.14159*2 + x[iter]
    return x

# 读取数据
# Read data from file
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_0P3.txt'
file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_1P4.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLG_ALPHA_1P4_BETA_0P0.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLG_ALPHA_1P4_BETA_0P3.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log_CLY_ALPHA_1P4_BETA_1P3.txt'
# file = '../data/log/JPC_TGyro_TGYRO_CBF_log.txt'

data = np.loadtxt(file)

# 设置起始和结束行数
start_line = 0
end_line = None
if end_line is None or end_line > len(data):
    end_line = len(data)

selected_data = data[start_line:end_line]

# 解析数据
px_mpc = selected_data[:,0] 
py_mpc = selected_data[:,1] 
pz_mpc = selected_data[:,2] 
rx = selected_data[:,3]
ry = selected_data[:,4]
rz = selected_data[:,5]
vx = selected_data[:,6]
vy = selected_data[:,7]
vz = selected_data[:,8]
gx = selected_data[:,9]
gy = selected_data[:,10]
gz = selected_data[:,11]
in1 = selected_data[:,12]
in2 = selected_data[:,13]
in3 = selected_data[:,14]
in4 = selected_data[:,15]

px_ref = selected_data[:,16] 
py_ref = selected_data[:,17] 
pz_ref = selected_data[:,18] 
rx_ref = selected_data[:,19]
ry_ref = selected_data[:,20]
rz_ref = selected_data[:,21]
vx_ref = selected_data[:,22]
vy_ref = selected_data[:,23]
vz_ref = selected_data[:,24]
gx_ref = selected_data[:,25]
gy_ref = selected_data[:,26]
gz_ref = selected_data[:,27]
time_cost = selected_data[:,32]

# 障碍物数据
obs_px = selected_data[:,33]
obs_py = selected_data[:,34]
obs_pz = selected_data[:,35]
obs_vx = selected_data[:,36]
obs_vy = selected_data[:,37]
obs_vz = selected_data[:,38]

# 创建时间轴
time_steps = np.arange(len(px_mpc))

print(f"绘制数据范围: 第 {start_line} 行到第 {end_line} 行")
print(f"总共 {len(px_mpc)} 个数据点")

# 函数：创建3D轨迹动画（在线播放）
def create_3d_animation_online():
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 设置坐标轴范围
    margin = 0.5
    ax.set_xlim(min(np.min(px_mpc), np.min(obs_px)) - margin, 
                max(np.max(px_mpc), np.max(obs_px)) + margin)
    ax.set_ylim(min(np.min(py_mpc), np.min(obs_py)) - margin, 
                max(np.max(py_mpc), np.max(obs_py)) + margin)
    ax.set_zlim(min(np.min(pz_mpc), np.min(obs_pz)) - margin, 
                max(np.max(pz_mpc), np.max(obs_pz)) + margin)
    
    ax.set_xlabel('X Position (m)', fontsize=10, fontname='Times New Roman')
    ax.set_ylabel('Y Position (m)', fontsize=10, fontname='Times New Roman')
    ax.set_zlabel('Z Position (m)', fontsize=10, fontname='Times New Roman')
    ax.set_title('3D Trajectory Animation (Online)', fontsize=12, fontname='Times New Roman')
    
    # 初始化轨迹线
    robot_line, = ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7, label='Robot Path')
    obstacle_line, = ax.plot([], [], [], 'r-', linewidth=2, alpha=0.7, label='Obstacle Path')
    reference_line, = ax.plot(px_ref, py_ref, pz_ref, 'g--', linewidth=1.5, alpha=0.5, label='Reference')
    
    # 初始化点
    robot_point, = ax.plot([], [], [], 'bo', markersize=8, label='Robot')
    obstacle_point, = ax.plot([], [], [], 'ro', markersize=8, label='Obstacle')
    
    # 轨迹历史
    robot_traj_x, robot_traj_y, robot_traj_z = [], [], []
    obstacle_traj_x, obstacle_traj_y, obstacle_traj_z = [], [], []
    
    # 添加距离文本
    distance_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes, fontsize=10,
                             fontname='Times New Roman', bbox=dict(boxstyle="round,pad=0.3", 
                             facecolor="white", alpha=0.8))
    
    # 添加时间文本
    time_text = ax.text2D(0.05, 0.90, '', transform=ax.transAxes, fontsize=10,
                         fontname='Times New Roman', bbox=dict(boxstyle="round,pad=0.3", 
                         facecolor="white", alpha=0.8))
    
    # 添加控制按钮说明
    control_text = ax.text2D(0.05, 0.85, 'Press SPACE to pause/resume', transform=ax.transAxes, 
                            fontsize=8, fontname='Times New Roman', color='red')
    
    ax.legend(prop={'family': 'Times New Roman', 'size': 8})
    
    def init():
        robot_line.set_data([], [])
        robot_line.set_3d_properties([])
        obstacle_line.set_data([], [])
        obstacle_line.set_3d_properties([])
        robot_point.set_data([], [])
        robot_point.set_3d_properties([])
        obstacle_point.set_data([], [])
        obstacle_point.set_3d_properties([])
        distance_text.set_text('')
        time_text.set_text('')
        return robot_line, obstacle_line, robot_point, obstacle_point, distance_text, time_text
    
    def animate(i):
        # 更新机器人轨迹
        robot_traj_x.append(px_mpc[i])
        robot_traj_y.append(py_mpc[i])
        robot_traj_z.append(pz_mpc[i])
        robot_line.set_data(robot_traj_x, robot_traj_y)
        robot_line.set_3d_properties(robot_traj_z)
        
        # 更新障碍物轨迹
        obstacle_traj_x.append(obs_px[i])
        obstacle_traj_y.append(obs_py[i])
        obstacle_traj_z.append(obs_pz[i])
        obstacle_line.set_data(obstacle_traj_x, obstacle_traj_y)
        obstacle_line.set_3d_properties(obstacle_traj_z)
        
        # 更新点位置
        robot_point.set_data([px_mpc[i]], [py_mpc[i]])
        robot_point.set_3d_properties([pz_mpc[i]])
        
        obstacle_point.set_data([obs_px[i]], [obs_py[i]])
        obstacle_point.set_3d_properties([obs_pz[i]])
        
        # 计算距离
        distance = np.sqrt((px_mpc[i] - obs_px[i])**2 + 
                          (py_mpc[i] - obs_py[i])**2 + 
                          (pz_mpc[i] - obs_pz[i])**2)
        
        # 更新文本
        distance_text.set_text(f'Distance: {distance:.3f} m')
        time_text.set_text(f'Time Step: {i}')
        
        return robot_line, obstacle_line, robot_point, obstacle_point, distance_text, time_text
    
    # 创建动画（不保存）
    ani = FuncAnimation(fig, animate, frames=len(px_mpc), init_func=init, 
                       interval=50, blit=True, repeat=True)
    
    # 添加暂停/继续功能
    def on_key(event):
        if event.key == ' ':
            if ani.event_source:
                ani.event_source.stop()
            else:
                ani.event_source.start()
    
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    plt.tight_layout()
    print("3D轨迹动画正在在线播放...按SPACE键暂停/继续")
    plt.show()
    
    return ani

# 函数：创建速度动画（在线播放）
def create_velocity_animation_online():
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    axes = axes.flatten()
    
    # 设置子图标题
    titles = ['Robot Velocity', 'Obstacle Velocity', 'Target Velocity', 'Relative Distance']
    ylabels = ['Velocity (m/s)', 'Velocity (m/s)', 'Velocity (m/s)', 'Distance (m)']
    
    for ax, title, ylabel in zip(axes, titles, ylabels):
        ax.set_xlabel('Time Step', fontsize=8, fontname='Times New Roman')
        ax.set_ylabel(ylabel, fontsize=8, fontname='Times New Roman')
        ax.set_title(title, fontsize=10, fontname='Times New Roman')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, len(px_mpc))
    
    # 设置y轴范围
    all_velocities = np.concatenate([vx, vy, vz, obs_vx, obs_vy, obs_vz, vx_ref, vy_ref, vz_ref])
    vel_min, vel_max = np.min(all_velocities), np.max(all_velocities)
    axes[0].set_ylim(vel_min - 0.1, vel_max + 0.1)
    axes[1].set_ylim(vel_min - 0.1, vel_max + 0.1)
    axes[2].set_ylim(vel_min - 0.1, vel_max + 0.1)
    
    # 初始化线条
    lines = []
    # 机器人速度
    line1, = axes[0].plot([], [], 'r-', label='Vx', linewidth=1.5)
    line2, = axes[0].plot([], [], 'g-', label='Vy', linewidth=1.5)
    line3, = axes[0].plot([], [], 'b-', label='Vz', linewidth=1.5)
    lines.extend([line1, line2, line3])
    
    # 障碍物速度
    line4, = axes[1].plot([], [], 'r-', label='Vx', linewidth=1.5)
    line5, = axes[1].plot([], [], 'g-', label='Vy', linewidth=1.5)
    line6, = axes[1].plot([], [], 'b-', label='Vz', linewidth=1.5)
    lines.extend([line4, line5, line6])
    
    # 目标速度
    line7, = axes[2].plot([], [], 'r-', label='Vx', linewidth=1.5)
    line8, = axes[2].plot([], [], 'g-', label='Vy', linewidth=1.5)
    line9, = axes[2].plot([], [], 'b-', label='Vz', linewidth=1.5)
    lines.extend([line7, line8, line9])
    
    # 相对距离
    line10, = axes[3].plot([], [], 'k-', linewidth=2, label='Distance')
    lines.append(line10)
    
    # 添加图例
    for ax in axes[:3]:
        ax.legend(prop={'family': 'Times New Roman', 'size': 7})
    axes[3].legend(prop={'family': 'Times New Roman', 'size': 7})
    
    # 时间文本
    time_text = fig.text(0.5, 0.01, '', ha='center', fontsize=10, 
                        fontname='Times New Roman', bbox=dict(boxstyle="round,pad=0.3", 
                        facecolor="white", alpha=0.8))
    
    def init():
        for line in lines:
            line.set_data([], [])
        time_text.set_text('')
        return lines + [time_text]
    
    def animate(i):
        # 更新机器人速度
        lines[0].set_data(time_steps[:i+1], vx[:i+1])
        lines[1].set_data(time_steps[:i+1], vy[:i+1])
        lines[2].set_data(time_steps[:i+1], vz[:i+1])
        
        # 更新障碍物速度
        lines[3].set_data(time_steps[:i+1], obs_vx[:i+1])
        lines[4].set_data(time_steps[:i+1], obs_vy[:i+1])
        lines[5].set_data(time_steps[:i+1], obs_vz[:i+1])
        
        # 更新目标速度
        lines[6].set_data(time_steps[:i+1], vx_ref[:i+1])
        lines[7].set_data(time_steps[:i+1], vy_ref[:i+1])
        lines[8].set_data(time_steps[:i+1], vz_ref[:i+1])
        
        # 更新相对距离
        distances = []
        for j in range(i+1):
            dist = np.sqrt((px_mpc[j] - obs_px[j])**2 + 
                          (py_mpc[j] - obs_py[j])**2 + 
                          (pz_mpc[j] - obs_pz[j])**2)
            distances.append(dist)
        lines[9].set_data(time_steps[:i+1], distances)
        
        # 设置距离y轴范围
        if len(distances) > 0:
            axes[3].set_ylim(max(0, min(distances) - 0.1), max(distances) + 0.1)
        
        time_text.set_text(f'Time Step: {i}')
        
        return lines + [time_text]
    
    # 创建动画（不保存）
    ani = FuncAnimation(fig, animate, frames=len(px_mpc), init_func=init, 
                       interval=50, blit=True, repeat=True)
    
    # 添加暂停/继续功能
    def on_key(event):
        if event.key == ' ':
            if ani.event_source:
                ani.event_source.stop()
            else:
                ani.event_source.start()
    
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.97])
    print("速度动画正在在线播放...按SPACE键暂停/继续")
    plt.show()
    
    return ani

# 主程序
if __name__ == "__main__":
    # 选择要播放的动画（取消注释其中一个）
    
    # 播放3D轨迹动画
    print("正在启动3D轨迹动画...")
    ani_3d = create_3d_animation_online()
    
    # 播放速度动画
    # print("正在启动速度动画...")
    # ani_velocity = create_velocity_animation_online()
    
    # 如果想要同时播放两个动画，需要分别运行（matplotlib通常一次只能显示一个动画窗口）