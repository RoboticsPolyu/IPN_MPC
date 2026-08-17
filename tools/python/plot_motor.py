import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation
column = 1

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

file = '/Users/ypwen/IPN/IPN_MPC/data/motor_calib_debug_log.txt'

# data1.pwm << " " << data1.rotor_speed1 << " " << data2.rotor_speed1 - data1.rotor_speed1 
data = np.loadtxt(file)

def savefig(plt, path):
        cm_to_inc = 1 / 2.54  # 厘米和英寸的转换 1inc = 2.54cm
        gcf = plt.gcf()  # 获取当前图像
        if column==1:
                gcf.set_size_inches(7 * cm_to_inc, 4.33 * cm_to_inc)
        else:
                gcf.set_size_inches(14.33 * cm_to_inc, 4.33 * cm_to_inc)
        plt.savefig(path, dpi=600, bbox_inches='tight', format='svg')

err = data[:,0]

pwm = data[:,1] 
rotor_speed1 = data[:,2] 
rotor_diff = data[:,3]

plt.figure()
plt.plot(rotor_speed1,  color = 'darkred', linestyle='-')
plt.xlabel('Time (5 ms)')
plt.ylabel('Rotor speed (RPM)')
plt.grid()
plt.title('Rotor speed measurements')
savefig(plt, 'figures/Rotor speed measurements.svg')

plt.figure()
plt.plot(rotor_speed1,  color = 'darkred', linestyle='-')
plt.xlabel('Time (5 ms)')
plt.ylabel('Rotor speed (RPM)')
plt.grid()
plt.xlim(2220, 2320)
plt.ylim(14000, 18000)
plt.title('Rotor speed measurements')
savefig(plt, 'figures/Rotor speed measurements part.svg')

plt.figure()
plt.plot(rotor_diff,  color = 'darkblue', linestyle='-')
plt.plot(err,  color = 'darkred', linestyle='-.')
plt.legend(['drpm', 'prediction error'])
plt.xlabel('Time (5 ms)')
plt.ylabel('Rotor speed (RPM)')
plt.grid()
plt.xlim(1100, 1200)
# plt.ylim(14000, 18000)
plt.title('Motor Calibration')
savefig(plt, 'figures/motor calibration.svg')

plt.show()