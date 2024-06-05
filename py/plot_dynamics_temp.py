import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation


def square_elements(arr):
    result = []
    for num in arr:
        result.append(num ** 2)
    return result
    
# font = {'family' : 'Times New Roman',
#         #'weight' : 'bold',
#         'size'   : 8}

# plt.rc('font', **font)


# plt.rcParams['figure.dpi'] = 300

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

# 设置输出格式为PDF
# plt.rcParams['savefig.format'] = 'pdf'

        # calib_log << std::setprecision(19) << Interp_states.at(idx).timestamp << std::setprecision(5) 
        # << " " << dyn_pos_err(0) << " " << dyn_pos_err(1) << " " << dyn_pos_err(2) << " " << dyn_e(3) << " " << dyn_e(4) << " " << dyn_e(5) << " " << dyn_e(6) 
        # << " " << dyn_e(7) << " " << dyn_e(8) << " " << dyn_e(9) << " " << dyn_e(10) << " " << dyn_e(11) 
        # << " " << thrust_torque(0)<< " " << thrust_torque(1) << " " << thrust_torque(2) << " " << thrust_torque(3) << " " << thrust_torque(4) << " " << thrust_torque(5) 
        # << " " << vel_body(0) << " " << vel_body(1) << " " << vel_body(2) 
        # << " " << Interp_states.at(idx).omega.x() << " " << Interp_states.at(idx).omega.y() << " " << Interp_states.at(idx).omega.z() 
file = '/Users/ypwen/IPN/IPN_MPC/data/calib_debug_log_quad13.txt'

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


rms_x_thrust = np.sqrt(np.mean((dyn_vel_err_x *100)**2))
print('-- Z-axis Force rms ', rms_x_thrust)
rms_y_thrust = np.sqrt(np.mean((dyn_vel_err_y *100)**2))
print('-- Y-axis Force rms ', rms_y_thrust)
rms_z_thrust = np.sqrt(np.mean((dyn_vel_err_z *100)**2))
print('-- Z-axis Force rms ', rms_z_thrust)

rms_x_omega = np.sqrt(np.mean((dyn_ome_err_x *100)**2))
print('-- Z-axis Torque rms ', rms_x_omega)
rms_y_omega = np.sqrt(np.mean((dyn_ome_err_y *100)**2))
print('-- Y-axis Torque rms ', rms_y_omega)
rms_z_omega = np.sqrt(np.mean((dyn_ome_err_z *100)**2))
print('-- Z-axis Torque rms ', rms_z_omega)

x_lower = 0
x_upper = 30
column = 1

# plt.figure(2)
# plt.plot((timestamp - timestamp[0]), dyn_vel_err_x *100, color = 'darkred', linestyle='-')
# plt.plot((timestamp - timestamp[0]), dyn_vel_err_y *100, color = 'darkblue', linestyle='-')
# plt.plot((timestamp - timestamp[0]), dyn_vel_err_z *100, color = 'purple', linestyle='-')
# plt.legend(['body-x', 'body-y', 'body-z'])
# plt.xlabel('Time (s)') #注意后面的字体属性
# plt.ylabel('Force (N)')
# plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
# plt.title('Predicted force error')  

# cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
# gcf = plt.gcf()  # 获取当前图像
# if column==1:
#         gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
# else:
#         gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)

plt.figure(3)
plt.plot((timestamp - timestamp[0]), thrust_ez, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), thrust_ez + dyn_vel_err_z *100, color = 'darkblue', linestyle='--')
plt.legend(['Predicted force', 'Force GT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Body-z force and ground truth (GT)')  
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
plt.savefig('figures/Body-z force and ground truth (GT).svg', dpi=600, bbox_inches='tight', format='svg') 


plt.figure(4)
plt.plot((timestamp - timestamp[0]), thrust_ez, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), thrust_ez + dyn_vel_err_z *100, color = 'darkblue', linestyle='--')
plt.legend(['Predicted thrust', 'Thrust GT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted thrust and ground truth (GT)')  
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
plt.savefig('figures/Predicted thrust and ground truth (GT).svg', dpi=600, bbox_inches='tight', format='svg') 

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
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
plt.savefig('figures/Predicted torque.svg', dpi=600, bbox_inches='tight', format='svg') 


plt.figure(6)
plt.plot((timestamp - timestamp[0]), tq_ex, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ex + dyn_ome_err_x*100, color = 'darkblue', linestyle='--')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['x', 'GT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted x-axis torque and ground truth (GT)')  
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)

plt.savefig('figures/Predicted x-axis torque and ground truth (GT).svg', dpi=600, bbox_inches='tight', format='svg') 



plt.figure(7)
plt.plot((timestamp - timestamp[0]), tq_ey, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), tq_ey + dyn_ome_err_y*100, color = 'darkblue', linestyle='--')
# plt.plot((timestamp - timestamp[0]), hor_thrust, color = 'black', linestyle='-')
plt.legend(['y', 'GT'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Torque (N*m)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted y-axis torque and ground truth (GT)')  
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)

plt.savefig('figures/Predicted y-axis torque and ground truth (GT).svg', dpi=600, bbox_inches='tight', format='svg') 

plt.figure(8)
plt.plot((timestamp - timestamp[0]), dyn_vel_err_x*100, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_y*100, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0]), dyn_vel_err_z*100, color = 'purple', linestyle='-')
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Error (m/s)')
plt.grid(ls='--')
plt.xlim((x_lower, x_upper))
plt.title('Predicted force errors')  
cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
gcf = plt.gcf()  # 获取当前图像
if column==1:
        gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
else:
        gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
plt.savefig('figures/Predicted force errors.svg', dpi=600, bbox_inches='tight', format='svg') 






plt.show()