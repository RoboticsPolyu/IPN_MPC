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
plt.rcParams['axes.labelsize'] = 6  # 坐标轴标签字体大小
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

file = '/Users/ypwen/IPN/IPN_MPC/data/calib_debug_log.txt'

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


body_wx = data[:,19]
body_wy = data[:,20]
body_wz = data[:,21]

df_ex = data[:,25]
df_ey = data[:,26]
df_ez = data[:,27]

hor_thrust = data[:,28]

vt_x = data[:,29]
vt_y = data[:,30]
vt_z = data[:,31]


freq = 400
# body_vel_x_gt = data[:,29]
# body_vel_y_gt = data[:,30]
# body_vel_z_gt = data[:,31]

rms_x_thrust = np.sqrt(np.mean((dyn_vel_err_x * freq)**2))
print('-- x-axis Force rms ', rms_x_thrust)
rms_y_thrust = np.sqrt(np.mean((dyn_vel_err_y * freq)**2))
print('-- Y-axis Force rms ', rms_y_thrust)
rms_z_thrust = np.sqrt(np.mean((dyn_vel_err_z * freq)**2))
print('-- Z-axis Force rms ', rms_z_thrust)

rms_x_omega = np.sqrt(np.mean((dyn_ome_err_x * freq)**2))
print('-- Z-axis Torque rms ', rms_x_omega)
rms_y_omega = np.sqrt(np.mean((dyn_ome_err_y * freq)**2))
print('-- Y-axis Torque rms ', rms_y_omega)
rms_z_omega = np.sqrt(np.mean((dyn_ome_err_z * freq)**2))
print('-- Z-axis Torque rms ', rms_z_omega)

x_lower = 0
x_upper = 20
column  = 1

plt.figure(1)
plt.plot((timestamp - timestamp[0]), df_ex, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), df_ey, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), df_ez, color = 'purple', linestyle='-')
plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['Predicted body-x drag force', 'Predicted body-y drag force', 'Predicted body-z drag force', 'HVT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.ylim((-3, 6.5))
plt.xlim((x_lower, x_upper))
plt.title('Predicted drag force and HVT')  
savefig(plt,'figures/Predicted drag force and HVT.svg')


plt.figure(2)
plt.plot((timestamp - timestamp[0]), dyn_vel_err_x * freq, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_y * freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_z * freq, color = 'purple', linestyle='-')
plt.legend(['body-x', 'body-y', 'body-z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted force error')  


plt.figure(3)
plt.plot((timestamp - timestamp[0]), hor_thrust + thrust_ez + df_ez + dyn_vel_err_z * freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), hor_thrust + thrust_ez + df_ez, color = 'darkred', linestyle='--')
plt.legend(['Force GT', 'Predicted force'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Body-z force and ground truth (GT)')  
savefig(plt,'figures/Body-z force and ground truth (GT).svg')


plt.figure(4)
plt.plot((timestamp - timestamp[0]), hor_thrust + thrust_ez, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), hor_thrust + thrust_ez + df_ez + dyn_vel_err_z * freq, color = 'darkblue', linestyle='--')
plt.legend(['Predicted thrust', 'Thrust GT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted thrust and ground truth (GT)')  
savefig(plt,'figures/Predicted thrust and ground truth (GT).svg')


plt.figure(5)
plt.plot((timestamp - timestamp[0]), tq_ex, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ey, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ez, color = 'purple', linestyle='-')
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted torque')  
savefig(plt,'figures/Predicted torque.svg')


plt.figure(6)
plt.plot((timestamp - timestamp[0]), tq_ex - vt_x + dyn_ome_err_x* freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ex - vt_x, color = 'darkred', linestyle='--')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['GT', 'x'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted x-axis torque and ground truth (GT)')  
savefig(plt,'figures/Predicted x-axis torque and ground truth (GT).svg')


plt.figure(7)
plt.plot((timestamp - timestamp[0]), tq_ey - vt_y + dyn_ome_err_y* freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ey - vt_y, color = 'darkred', linestyle='--')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['GT','y'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted y-axis torque and ground truth (GT)')  
savefig(plt,'figures/Predicted y-axis torque and ground truth (GT).svg')


plt.figure(8)
plt.plot((timestamp - timestamp[0]), dyn_vel_err_x* freq, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_y* freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_z* freq, color = 'purple', linestyle='-')
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Error (m/s)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted force errors')  
savefig(plt,'figures/Predicted force errors.svg')


plt.figure(9)
plt.plot((timestamp - timestamp[0]), body_vel_x, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_vel_y, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_vel_z, color = 'black', linestyle='-')
c=[np.square(body_vel_x)[i]+np.square(body_vel_y)[i] +np.square(body_vel_z)[i] for i in range(min(len(np.square(body_vel_y)),len(np.square(body_vel_y))))]
plt.plot((timestamp - timestamp[0]), c, color = 'black', linestyle='-')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['x', 'y', 'z','x*x+y*y'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Velocity (m/s)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Body velocity')  
savefig(plt,'figures/Body velocity.svg')

plt.figure(10)
plt.plot((timestamp - timestamp[0]), tq_ez - vt_z + dyn_ome_err_z* freq, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ez - vt_z , color = 'darkred', linestyle='--')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['GT', 'z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted z-axis torque and ground truth (GT)')  
savefig(plt,'figures/Predicted z-axis torque and ground truth (GT).svg')

plt.figure(11)
plt.plot((timestamp - timestamp[0]), body_wx, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_wy, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), body_wz, color = 'black', linestyle='-')

# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Angular Velocity (rad/s)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Angular velocity')  
savefig(plt,'figures/Angular velocity.svg')

# plt.figure(11)
# plt.plot((timestamp - timestamp[0]), body_vel_x, color = 'darkred', linestyle='-.')
# plt.plot((timestamp - timestamp[0]), body_vel_y, color = 'darkblue', linestyle='-.')
# plt.plot((timestamp - timestamp[0]), body_vel_z, color = 'black', linestyle='-.')

# plt.plot((timestamp - timestamp[0]), body_vel_x_gt, color = 'darkred', linestyle='-')
# plt.plot((timestamp - timestamp[0]), body_vel_y_gt, color = 'darkblue', linestyle='-')
# plt.plot((timestamp - timestamp[0]), body_vel_z_gt, color = 'black', linestyle='-')
# c=[np.square(body_vel_x_gt)[i]+np.square(body_vel_y_gt)[i] for i in range(min(len(np.square(body_vel_x_gt)),len(np.square(body_vel_y_gt))))]
# plt.plot((timestamp - timestamp[0]), c, color = 'black', linestyle='-')
# # plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
# plt.legend(['x', 'y', 'z','x', 'y', 'z','x*x+y*y'])
# plt.xlabel('Time (s)') #注意后面的字体属性
# plt.ylabel('Velocity (m/s)')
# plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
# plt.title('Body velocity GT')  

plt.show()