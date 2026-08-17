import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation

font = {'family' : 'Times New Roman',
        #'weight' : 'bold',
        'size'   : 8}

plt.rc('font', **font)

# POS_SIGMA 0.01m MPC Precise
# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_onlyGNSS_5_1__log.txt'

# data = np.loadtxt(file)

# px_mpc = data[:,0] 
# py_mpc = data[:,1] 
# pz_mpc = data[:,2] 
# rx = data[:,3]
# ry = data[:,4]
# rz = data[:,5]
# vx = data[:,6]
# vy = data[:,7]
# vz = data[:,8]
# gx = data[:,9]
# gy = data[:,10]
# gz = data[:,11]

# px_ref = data[:,16] 
# py_ref = data[:,17] 
# pz_ref = data[:,18] 
# rx_ref = data[:,19]
# ry_ref = data[:,20]
# rz_ref = data[:,21]
# vx_ref = data[:,22]
# vy_ref = data[:,23]
# vz_ref = data[:,24]
# gx_ref = data[:,25]
# gy_ref = data[:,26]
# gz_ref = data[:,27]

# px_mpc_err = px_mpc - px_ref
# MSEX = np.square(px_mpc_err).mean() 
# RMSEX = math.sqrt(MSEX)

# py_mpc_err = py_mpc - py_ref
# MSEY = np.square(py_mpc - py_ref).mean() 
# RMSEY = math.sqrt(MSEY)

# pz_mpc_err = pz_mpc - pz_ref
# MSEZ = np.square(pz_mpc - pz_ref).mean() 
# RMSEZ = math.sqrt(MSEZ)

# print("MPC RMSEX: %f", RMSEX)
# print("MPC RMSEX: %f", RMSEY)
# print("MPC RMSEX: %f", RMSEZ)


# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_JPCM_TEST_D0.1_log.txt'

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


# plt.figure(figsize=(5, 4))
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.plot(px_jpcm[0:1000], py_jpcm[0:1000], '-.b', linewidth =.5)
# plt.plot(px_mpc[0:1000], py_mpc[0:1000],  '-r', linewidth =.5)
# plt.xlabel('x-axis') #注意后面的字体属性
# plt.ylabel('y-axis')
# plt.legend(['JPCM', 'MPC'])
# plt.title('Path comparison')  
# plt.grid()
# plt.savefig('Path.png',dpi=600, bbox_inches='tight')


px_jpcm_err = px_jpcm - px_ref
MSEX = np.square(px_jpcm_err).mean() 
RMSEX = math.sqrt(MSEX)

py_jpcm_err = py_jpcm - py_ref
MSEY = np.square(py_jpcm_err).mean() 
RMSEY = math.sqrt(MSEY)

pz_jpcm_err = pz_jpcm - pz_ref
MSEZ = np.square(pz_jpcm_err).mean() 
RMSEZ = math.sqrt(MSEZ)

rx_jpcm_err = rx_jpcm - rx_ref
rMSEX = np.square(px_jpcm_err).mean() 
rRMSEX = math.sqrt(rMSEX)

ry_jpcm_err = ry_jpcm - ry_ref
rMSEY = np.square(ry_jpcm_err).mean() 
rRMSEY = math.sqrt(rMSEY)

rz_jpcm_err = rz_jpcm - rz_ref
rMSEZ = np.square(rz_jpcm_err).mean() 
rRMSEZ = math.sqrt(rMSEZ)


print("JPCM RMSEX: %f", RMSEX)
print("JPCM RMSEX: %f", RMSEY)
print("JPCM RMSEX: %f", RMSEZ)

print("JPCM rRMSEX: %f", rRMSEX)
print("JPCM rRMSEX: %f", rRMSEY)
print("JPCM rRMSEX: %f", rRMSEZ)

plt.figure(figsize=(5, 2.5))
# plt.subplot(2, 1, 1)
plt.grid()
# px_mpc = px_mpc[0:1:3000]
# py_mpc = py_mpc[0:1:3000]
plt.plot(px_jpcm_err, '-r', linewidth =1)
plt.plot(py_jpcm_err,  '-g', linewidth =1)
plt.plot(pz_jpcm_err,  '-b', linewidth =1)
plt.legend(['x', 'y', 'z'])
plt.ylabel('residuals') #注意后面的字体属性
plt.xlabel('time(0.01s)')
plt.title('Postion following residuals with drag force based on JPCM')  

# plt.subplot(2, 1, 2)
# # px_mpc = px_mpc[0:1:3000]
# # py_mpc = py_mpc[0:1:3000]
# plt.grid()
# plt.plot(px_mpc_err, '-r', linewidth =1)
# plt.plot(py_mpc_err,  '-g', linewidth =1)
# plt.plot(pz_mpc_err,  '-b', linewidth =1)
# plt.legend(['x', 'y', 'z'])
# plt.ylabel('residuals') #注意后面的字体属性
# plt.xlabel('time(0.01s)')
# plt.title('Postion following residuals based on MPC')  
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
plt.savefig('Drag.png',dpi=600, bbox_inches='tight')

plt.show()


