import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation

def savefig(plt, path):
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
plt.rcParams['font.sans-serif'] = ['Times New Roman']  # 如果要显示中文字体,则在此处设为：simhei,Arial Unicode MS
plt.rcParams['font.weight'] = 'light'
plt.rcParams['axes.unicode_minus'] = False  # 坐标轴负号显示
plt.rcParams['axes.titlesize'] = 8  # 标题字体大小
plt.rcParams['axes.labelsize'] = 7  # 坐标轴标签字体大小
plt.rcParams['xtick.labelsize'] = 7  # x轴刻度字体大小
plt.rcParams['ytick.labelsize'] = 7  # y轴刻度字体大小
plt.rcParams['legend.fontsize'] = 6

# 线条调整
plt.rcParams['axes.linewidth'] = 0.5

# 刻度在内，设置刻度字体大小
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'

# calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
# << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) 
# << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
# << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
# << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) 
# << " " << Interp_states.at(idx).omega.x() << " " << Interp_states.at(idx).omega.y() << " " << Interp_states.at(idx).omega.z() 

file = '/Users/ypwen/IPN/IPN_MPC/data/calib_debug_log_neuro.txt'

data = np.loadtxt(file)

timestamp = data[:,0]

dyn_pos_err_x = data[:,1] 
dyn_pos_err_y = data[:,2] 
dyn_pos_err_z = data[:,3]

dyn_vel_err_x = data[:,7] 
dyn_vel_err_y = data[:,8] 
dyn_vel_err_z = data[:,9]

dyn_ome_err_x = data[:,10] 
dyn_ome_err_y = data[:,11] 
dyn_ome_err_z = data[:,12]

thrust_ex = data[:,13]
thrust_ey = data[:,14]
thrust_ez = data[:,15]

tq_ex = data[:,16]
tq_ey = data[:,17]
tq_ez = data[:,18]

body_vel_x = data[:,19]
body_vel_y = data[:,20]
body_vel_z = data[:,21]

df_ex = data[:,25]
df_ey = data[:,26]
df_ez = data[:,27]

hor_thrust = data[:,28]

file = '/Users/ypwen/IPN/IPN_MPC/data/calib_debug_log_black.txt'

data = np.loadtxt(file)
timestampp = data[:,0]
body_vel_xx = data[:,19]
body_vel_yy = data[:,20]
body_vel_zz = data[:,21]

hor_thrustt = data[:,28]

x_lower = 0
x_upper = 20
column  = 1

fig = plt.figure()
plt.plot((timestamp - timestamp[0]), body_vel_x, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_vel_y, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_vel_z, color = 'green', linestyle='-')
plt.legend(['vx', 'vy', 'vz'])
plt.xlabel('Time (s)')
plt.ylabel('Vel (m/s)')
plt.grid
plt.title('Body veclocity')


fig = plt.figure()
c=[np.square(body_vel_x)[i]+np.square(body_vel_y)[i] for i in range(min(len(np.square(body_vel_y)),len(np.square(body_vel_y))))]
cc=[np.square(body_vel_xx)[i]+np.square(body_vel_yy)[i] for i in range(min(len(np.square(body_vel_yy)),len(np.square(body_vel_yy))))]
ax = fig.add_subplot(111)

ax.plot((timestamp - timestamp[0]), c, color = 'darkred', linestyle='-', label='NeuroBEM $v_h^2$')
ax.plot((timestampp - timestampp[0]), cc, color = 'darkblue', linestyle='-', label='Blackbird $v_h^2$')
ax.legend(loc=(0.2, 0.3))
ax2 = ax.twinx()

ax2.plot((timestamp - timestamp[0]), hor_thrust, color = 'red', linestyle='dashed', label='NeuroBEM T')
ax2.plot((timestampp - timestampp[0]), hor_thrustt, color = 'blue', linestyle='dashed', label='Blackbird T')
ax2.legend(loc=(0.6, 0.25))
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
# plt.legend(['NeuroBEM', 'Blackbird'])
ax.set_xlabel('Time (s)') #注意后面的字体属性
ax.set_ylabel('Sum of squares of horizonal velocity (m/s)^2')
ax.grid(ls='--')
ax2.set_ylabel('Force (N)')
plt.xlim((x_lower, x_upper))
# ax.relim(-5, 100)
# plt.title('Sum of squares of horizonal velocity')  
plt.title('HVT')  
savefig(plt,'figures/HVT.svg')

plt.figure()
plt.plot(body_vel_z)
plt.plot(body_vel_zz)
plt.legend(['NeuroBEM', 'Blackbird'])

plt.show()