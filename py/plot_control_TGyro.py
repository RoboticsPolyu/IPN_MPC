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
file = '/Users/ypwen/IPN/IPN_MPC/data/JPC_TGyro_TGYRO_log.txt'

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

plt.figure()
plt.plot(rx, '-.g', linewidth =1)
plt.plot(ry, ':b', linewidth =1)
plt.plot(rz, '--y', linewidth =1)
plt.grid()
plt.legend(['rx', 'ry', 'rz'])
plt.title("Rotation")


plt.figure()

plt.plot(rx_ref, '-.g', linewidth =1)
plt.plot(ry_ref, ':b', linewidth =1)
plt.plot(rz_ref , '--y', linewidth =1)
plt.grid()
plt.legend(['rx', 'ry', 'rz'])
plt.title("Rotation Reference")

plt.figure()

plt.plot(gx_ref, '-.g', linewidth =1)
plt.plot(gy_ref, ':b', linewidth =1)
plt.plot(gz_ref, '--y', linewidth =1)
plt.grid()
plt.legend(['wx', 'wy', 'wz'])
plt.title("Angular speed reference")

plt.figure()
plt.plot(time_cost, '.r')
plt.xlabel("Index")
plt.ylabel("Time (s)")
plt.grid()
plt.title("Optimization time cost")
px_mpc_err = px_mpc - px_ref
MSEX = np.square(px_mpc_err).mean() 
RMSEX = math.sqrt(MSEX)

py_mpc_err = py_mpc - py_ref
MSEY = np.square(py_mpc - py_ref).mean() 
RMSEY = math.sqrt(MSEY)

pz_mpc_err = pz_mpc - pz_ref
MSEZ = np.square(pz_mpc - pz_ref).mean() 
RMSEZ = math.sqrt(MSEZ)

rx_mpc_err = limit(rx - rx_ref)
rMSEX = np.square(rx_mpc_err).mean() 
rRMSEX = math.sqrt(rMSEX)

ry_mpc_err = limit(ry - ry_ref)
rMSEY = np.square(ry_mpc_err).mean() 
rRMSEY = math.sqrt(rMSEY)

rz_mpc_err = limit(rz - rz_ref)
rMSEZ = np.square(rz_mpc_err).mean() 
rRMSEZ = math.sqrt(rMSEZ)


print("MPC RMSEX: %f", RMSEX)
print("MPC RMSEX: %f", RMSEY)
print("MPC RMSEX: %f", RMSEZ)

print("MPC rRMSEX: %f", rRMSEX)
print("MPC rRMSEX: %f", rRMSEY)
print("MPC rRMSEX: %f", rRMSEZ)


# plt.figure(figsize=(5, 2.5))
# plt.grid()
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_mpc_err, '-r', linewidth =1)
# plt.plot(py_mpc_err,  '-.g', linewidth =1)
# plt.plot(pz_mpc_err,  ':b', linewidth =1)
# # plt.ylim(-0.15, 0.15)
# plt.legend(['x', 'y', 'z'])
# plt.ylabel('residuals (m)') #注意后面的字体属性
# plt.xlabel('time (0.01s)')
# plt.title('Postion following residuals based on JPCM')  
# plt.tight_layout()
# plt.savefig('JPCM Position Drag=0.2.png',dpi=600, bbox_inches='tight')


# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '/Users/ypwen/IPN/IPN_MPC/data/JPC_JPCM_HIN_TEST_6_log.txt'

data = np.loadtxt(file)

px_jpcm = data[:,0] 
py_jpcm = data[:,1] 
pz_jpcm = data[:,2] 
rx_jpcm = data[:,3]
ry_jpcm = data[:,4]
rz_jpcm = data[:,5]
vx = data[:,6]
vy = data[:,7]
vz = data[:,8]
gx = data[:,9]
gy = data[:,10]
gz = data[:,11]

jin1 = data[:,12]
jin2 = data[:,13]
jin3 = data[:,14]
jin4 = data[:,15]

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


# plt.figure(figsize=(5, 4.5))
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_jpcm[0:1000], py_jpcm[0:1000], '-.b', linewidth =.5)
# plt.plot(px_mpc[0:1000], py_mpc[0:1000],  '-r', linewidth =.5)
# plt.xlabel('x-axis (m)') #注意后面的字体属性
# plt.ylabel('y-axis (m)')
# plt.legend(['JPCM', 'MPC'])
# plt.title('Path comparison')  
# plt.grid()
# plt.savefig('Path.png',dpi=600, bbox_inches='tight')

# plt.figure(figsize=(5, 4.5))
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_jpcm[0:1000], py_jpcm[0:1000], '-.b', linewidth =.5)
# plt.plot(px_mpc[0:1000], py_mpc[0:1000],  '-r', linewidth =.5)
# plt.xlabel('x-axis') #注意后面的字体属性
# plt.ylabel('y-axis')
# plt.legend(['JPCM-Drag', 'JPCM'])
# plt.title('Path comparison of MPC and JPCM')  
# plt.grid()
# plt.savefig('Path of JPCM-Drag and JPCM.png',dpi=600, bbox_inches='tight')

# plt.figure(figsize=(5, 4.5))
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_jpcm[0:300], py_jpcm[0:300], '-.b', linewidth =1.0)
# plt.xlabel('x-axis') #注意后面的字体属性
# plt.ylabel('y-axis')
# plt.title('Path of JPCM with drag force (-0.3)')  
# plt.grid()
# plt.savefig('Drag_0.3_Path.png',dpi=600, bbox_inches='tight')

px_jpcm_err = px_jpcm - px_ref
MSEX = np.square(px_jpcm_err).mean() 
RMSEX = math.sqrt(MSEX)

py_jpcm_err = py_jpcm - py_ref
MSEY = np.square(py_jpcm_err).mean() 
RMSEY = math.sqrt(MSEY)

pz_jpcm_err = pz_jpcm - pz_ref
MSEZ = np.square(pz_jpcm_err).mean() 
RMSEZ = math.sqrt(MSEZ)

rx_jpcm_err = limit(rx_jpcm - rx_ref)
rMSEX = np.square(rx_jpcm_err).mean() 
rRMSEX = math.sqrt(rMSEX)

ry_jpcm_err = limit(ry_jpcm - ry_ref)
rMSEY = np.square(ry_jpcm_err).mean() 
rRMSEY = math.sqrt(rMSEY)

rz_jpcm_err = limit(rz_jpcm - rz_ref)
rMSEZ = np.square(rz_jpcm_err).mean() 
rRMSEZ = math.sqrt(rMSEZ)


print("JPCM RMSEX: %f", RMSEX)
print("JPCM RMSEX: %f", RMSEY)
print("JPCM RMSEX: %f", RMSEZ)

print("JPCM rRMSEX: %f", rRMSEX)
print("JPCM rRMSEX: %f", rRMSEY)
print("JPCM rRMSEX: %f", rRMSEZ)

plt.figure(figsize=(5, 5))
plt.subplot(2, 1, 1)
plt.grid()
# px_mpc = px_mpc[0:1:3000]
# py_mpc = py_mpc[0:1:3000]
plt.plot(px_jpcm_err, '-r', linewidth =1)
plt.plot(py_jpcm_err,  '-.g', linewidth =1)
plt.plot(pz_jpcm_err,  ':b', linewidth =1)
plt.ylim(-0.10, 0.10)
plt.legend(['x', 'y', 'z'])
plt.ylabel('residuals (m)') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('Postion following residuals based on JPCM')  

plt.subplot(2, 1, 2)
# px_mpc = px_mpc[0:1:3000]
# py_mpc = py_mpc[0:1:3000]
plt.grid()
plt.plot(px_mpc_err, '-r', linewidth =1)
plt.plot(py_mpc_err,  '-.g', linewidth =1)
plt.plot(pz_mpc_err,  ':b', linewidth =1)
# plt.ylim(-0.40, 0.60)
plt.legend(['x', 'y', 'z'])
plt.ylabel('residuals (m)') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('Postion following residuals based on MPC')  
plt.tight_layout()
plt.savefig('MPC vs JPCM Position.png',dpi=600, bbox_inches='tight')

# plt.figure(figsize=(5, 2.5))
# plt.grid()
# plt.plot(px_mpc_err, '-r', linewidth =1)
# plt.plot(py_mpc_err,  '-.g', linewidth =1)
# plt.plot(pz_mpc_err,  ':b', linewidth =1)
# # plt.ylim(-0.40, 0.60)
# plt.legend(['x', 'y', 'z'])
# plt.ylabel('residuals (m)') #注意后面的字体属性
# plt.xlabel('time (0.01s)')
# plt.title('Postion following residuals based on MPC')  
# plt.tight_layout()
# plt.savefig('MPC Position.png',dpi=600, bbox_inches='tight')

# plt.figure(figsize=(5, 2.5))
# plt.plot(rx_mpc_err, '-r', linewidth =1)
# plt.plot(ry_mpc_err,  '-.g', linewidth =1)
# plt.plot(rz_mpc_err,  ':b', linewidth =1)
# # plt.ylim(-0.40, 0.60)
# plt.legend(['x', 'y', 'z'])
# plt.ylabel('residuals (m)') #注意后面的字体属性
# plt.xlabel('time (0.01s)')
# plt.title('Rotation following residuals based on MPC')  
# plt.tight_layout()
# plt.savefig('MPC Rotation.png',dpi=600, bbox_inches='tight')

# plt.figure(2)
# plt.plot(time - time[0], v_x, '-r', time- time[0], v_y, '-b', time- time[0], v_z, '-g')
# plt.legend(['v_x', 'v_y', 'v_z'])
# plt.xlabel('Time') #注意后面的字体属性
# plt.ylabel('Velocity')
# plt.title('Velocity')  


# # plt.savefig('PWM.jpg')
# ax = plt.figure().add_subplot(projection='3d')

# line = ax.plot(p_x, p_y, p_z, label='type curve')
# ax.plot(p_x[0:1], p_y[0:1], p_z[0:1], 'ro')
# ax.legend()
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')     
# plt.title('Position')   
plt.tight_layout()
plt.savefig('MPC vs JPCM.png',dpi=600, bbox_inches='tight')

# plt.figure(figsize=(5, 2.5))
# plt.grid()
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_jpcm_err, '-r', linewidth =1)
# plt.plot(py_jpcm_err,  '-.g', linewidth =1)
# plt.plot(pz_jpcm_err,  ':b', linewidth =1)
# # plt.ylim(-0.10, 0.10)
# plt.legend(['x', 'y', 'z'])
# plt.ylabel('residuals (m)') #注意后面的字体属性
# plt.xlabel('time (0.01s)')
# plt.title('Postion following residuals based on JPCM')  
# plt.tight_layout()
# plt.savefig('JPCM Position.png',dpi=600, bbox_inches='tight')


# plt.figure(figsize=(5, 2.5))
# plt.grid()
# plt.plot(rx_jpcm_err, '-r', linewidth =1)
# plt.plot(ry_jpcm_err,  '-.g', linewidth =1)
# plt.plot(rz_jpcm_err,  ':b', linewidth =1)
# # plt.ylim(-0.12, 0.20)
# plt.legend(['roll', 'pitch', 'yaw'])
# plt.ylabel('residuals (rad)') #注意后面的字体属性
# plt.xlabel('time (0.01s)')
# plt.title('Rotation following residuals based on JPCM') 

# plt.tight_layout()
# plt.savefig('JPCM Rotation.png',dpi=600, bbox_inches='tight')

plt.figure(figsize=(5, 4.5))
plt.subplot(2, 1, 1)
plt.grid(ls='--')
plt.plot(in1, '-r', linewidth =1)
plt.plot(in2, '-.g', linewidth =1)
plt.plot(in3, ':b', linewidth =1)
plt.plot(in4, '--y', linewidth =1)
plt.legend(['Thrust', 'wx', 'wy', 'wz'])
plt.ylabel('Value') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('MPC control input')
plt.subplot(2, 1, 2)
plt.grid(ls='--')
plt.plot(jin1, '-r', linewidth =1)
plt.plot(jin2, '-.g', linewidth =1)
plt.plot(jin3, ':b', linewidth =1)
plt.plot(jin4, '--y', linewidth =1)
# plt.ylim(12000, 19000)
plt.legend(['Thrust', 'wx', 'wy', 'wz'])
plt.ylabel('Value') #注意后面的字体属性
plt.xlabel('time (0.01s)')
plt.title('JPCM control input')
plt.tight_layout()

plt.show()



