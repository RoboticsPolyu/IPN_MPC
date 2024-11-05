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
plt.rcParams['axes.linewidth'] = 1

# 刻度在内，设置刻度字体大小
plt.rcParams['xtick.direction'] = 'in'
plt.rcParams['ytick.direction'] = 'in'


def limit(x):
  for iter in range(len(x)):
    if x[iter] > 3.14159:
      x[iter] = 3.14159*2-x[iter]
    if x[iter] < -3.14158:
      x[iter] = 3.14159*2 + x[iter]
  return x


# POS_SIGMA 0.01m MPC Precise
# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '../data/log/JPC_TGyro_TGYRO_log.txt'

data = np.loadtxt(file)

px_mpc = data[:,0] 
py_mpc = data[:,1] 
pz_mpc = data[:,2] 
rx = data[:,3]
ry = data[:,4]
rz = data[:,5]
vx = data[:,6]
vy = data[:,7]
vz = data[:,8]
gx = data[:,9]
gy = data[:,10]
gz = data[:,11]
in1 = data[:,12]
in2 = data[:,13]
in3 = data[:,14]
in4 = data[:,15]

px_ref = data[:,16] 
py_ref = data[:,17] 
pz_ref = data[:,18] 
rx_ref = data[:,19]
ry_ref = data[:,20]
rz_ref = data[:,21]
vx_ref = data[:,22]
vy_ref = data[:,23]
vz_ref = data[:,24]
gx_ref = data[:,25]
gy_ref = data[:,26]
gz_ref = data[:,27]
time_cost = data[:,32]

plt.figure(figsize=(5, 3))
plt.plot(px_mpc, '-.g', linewidth =1)
plt.plot(py_mpc, ':b', linewidth =1)
plt.plot(pz_mpc, '--y', linewidth =1)
plt.grid()
plt.legend(['P_x', 'P_y', 'P_z'])
plt.title("Positioning of Path")

plt.figure(figsize=(5, 3))
plt.plot(time_cost, '.r')
plt.xlabel("Index")
plt.ylabel("Time (s)")
plt.grid()
plt.title("Optimization time cost")

plt.figure(figsize=(5, 4.5))
plt.subplot(2, 1, 1)
plt.grid(ls='-')
plt.plot(in1, '-r', linewidth =1)
plt.ylabel('Force (N)') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('MPC control input')
plt.subplot(2, 1, 2)
plt.grid(ls='-')
plt.plot(in2, '-r', linewidth =1)
plt.plot(in3, '-g', linewidth =1)
plt.plot(in4, '-b', linewidth =1)
plt.legend(['wx', 'wy', 'wz'])
plt.ylabel('Angular velocity') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('MPC control input')
plt.tight_layout()

plt.show()



