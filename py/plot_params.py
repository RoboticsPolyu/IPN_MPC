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

file = '/Users/ypwen/IPN/IPN_MPC/data/batch_calib_params_log.txt' # for paper

# file = '/Users/ypwen/IPN/IPN_MPC/data/calib_params_log.txt'

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

time = data[:,0]
kf = data[:,1] 
km = data[:,2] 

dx = data[:,3]
dy = data[:,4] 
dz = data[:,5]

ax = data[:,6]
ay = data[:,7] 
az = data[:,8] 

bx = data[:,9]
by = data[:,10]
bz = data[:,11]

Rx = data[:,12]
Ry = data[:,13]
Rz = data[:,14]

Ix = data[:,15]
Iy = data[:,16]
Iz = data[:,17]

left = 100

# kf[left] = 2e-6

plt.figure()
plt.plot( kf[left:,], color = 'darkred', linestyle='-')
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameter $k_f$') 
savefig(plt, 'figures/p_kf.svg')

plt.figure()
plt.plot( km[left:,], color = 'darkblue', linestyle='-')
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameter $k_m$') 
savefig(plt, 'figures/p_km.svg')

plt.figure()
plt.plot( az[left:,], color = 'purple', linestyle='-')
# plt.legend(['kf', 'km', 'kv'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (N*s2/m2)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameter $k_h$')  
savefig(plt, 'figures/p_kh.svg')

plt.figure()
plt.plot( dx[left:,], color = 'darkred', linestyle='-')
# plt.xlabel('Batch index') #注意后面的字体属性
# plt.ylabel('Value (N*s/m)')
# plt.grid(ls='--')
# # plt.xlim((x_lower, x_upper))
# plt.title('Dynamic parameter $D_x$') 
# savefig(plt, path)

# plt.figure()
plt.plot( dy[left:,], color = 'darkblue', linestyle='-')
# plt.xlabel('Batch index') #注意后面的字体属性
# plt.ylabel('Value (N*s/m)')
# plt.grid(ls='--')
# # plt.xlim((x_lower, x_upper))
# plt.title('Dynamic parameter $D_y$') 
# savefig(plt, path)
plt.legend(['Dx', 'Dy'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (N*s/m)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters $D_x$ and $D_y$')
savefig(plt,  'figures/p_dxy.svg') 

plt.figure()
plt.plot( dz[left:,], color = 'purple', linestyle='-')
# plt.legend(['dx', 'dy', 'dz'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (N*s/m)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters $D_z$')
savefig(plt,  'figures/p_dz.svg') 


plt.figure()
plt.plot( bx[left:,], color = 'darkred', linestyle='-')
plt.plot( by[left:,], color = 'darkblue', linestyle='-')
plt.plot( bz[left:,], color = 'purple', linestyle='-')
plt.legend(['$B_x$', '$B_y$', '$B_z$'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters $B$') 
savefig(plt, 'figures/p_B.svg')

plt.figure()
plt.plot( ax[left:,], color = 'darkred', linestyle='-')
plt.plot( ay[left:,], color = 'darkblue', linestyle='-')
plt.legend(['x', 'y'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (m)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters CoG')
savefig(plt,  'figures/p_hog.svg') 


plt.figure()
plt.plot( Ix[left:,], color = 'darkred', linestyle='-')
plt.plot( Iy[left:,], color = 'darkblue', linestyle='-')
plt.legend(['$I_x$', '$I_y$'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (kg*m2)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters $I_x$ and $I_y$') 
savefig(plt, 'figures/p_Ixy.svg')

plt.figure()
plt.plot( Iz[left:,], color = 'purple', linestyle='-')
# plt.legend(['$I_x$', '$I_y$', '$I_z$'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (kg*m2)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters $I_z$') 
savefig(plt, 'figures/p_Iz.svg')


plt.figure()
plt.plot( Rx[left:,], color = 'darkred', linestyle='-')
plt.plot( Ry[left:,], color = 'darkblue', linestyle='-')
# plt.plot( Rz[left:,], color = 'purple', linestyle='-')
plt.legend(['$R_x$', '$R_y$'])
plt.xlabel('Batch index') #注意后面的字体属性
plt.ylabel('Value (kg*m2)')
plt.grid(ls='--')
# plt.xlim((x_lower, x_upper))
plt.title('Dynamic parameters gravity rotation') 
savefig(plt, 'figures/p_Rg.svg')


plt.show()