import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np
import math

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation

font = {'family' : 'Times New Roman',
        #'weight' : 'bold',
        'size'   : 9}

plt.rc('font', **font)



def limit(x):
  for iter in range(len(x)):
    if x[iter] > 3.14159:
      x[iter] = 3.14159*2-x[iter]
    if x[iter] < -3.14158:
      x[iter] = 3.14159*2 + x[iter]
  return x

# POS_SIGMA 0.01m MPC Precise
# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '/Users/ypwen/IPN/IPN_MPC/data/log/JEC_MPC_Rev_log.txt'

data = np.loadtxt(file)

px_mpc = data[:,0] 
py_mpc = data[:,1] 
pz_mpc = data[:,2] 
rx_mpc = data[:,3]
ry_mpc = data[:,4]
rz_mpc = data[:,5]
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

px_mpc_err = px_mpc - px_ref
MSEX = np.square(px_mpc_err).mean() 
RMSEX = math.sqrt(MSEX)

py_mpc_err = py_mpc - py_ref
MSEY = np.square(py_mpc - py_ref).mean() 
RMSEY = math.sqrt(MSEY)

pz_mpc_err = pz_mpc - pz_ref
MSEZ = np.square(pz_mpc - pz_ref).mean() 
RMSEZ = math.sqrt(MSEZ)

rx_mpc_err = limit(rx_mpc - rx_ref)
MSErX = np.square(rx_mpc_err).mean() 
RMSErX = math.sqrt(MSErX)

ry_mpc_err = limit(ry_mpc - ry_ref)
MSErY = np.square(ry_mpc - ry_ref).mean() 
RMSErY = math.sqrt(MSErY)

rz_mpc_err = limit(rz_mpc - rz_ref)
MSErZ = np.square(rz_mpc - rz_ref).mean() 
RMSErZ = math.sqrt(MSErZ)

print("MPC RMSEX: %f", RMSEX)
print("MPC RMSEX: %f", RMSEY)
print("MPC RMSEX: %f", RMSEZ)


# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '/Users/ypwen/IPN/IPN_MPC/data/log/JEC_JECM_Rev_0.20m_log.txt'

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


# plt.figure(figsize=(6, 5))
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

rx_jpcm_err = limit(rx_jpcm - rx_ref)
rMSEX = np.square(px_jpcm_err).mean() 
rRMSEX = math.sqrt(rMSEX)

ry_jpcm_err = limit(ry_jpcm - ry_ref)
rMSEY = np.square(ry_jpcm_err).mean() 
rRMSEY = math.sqrt(rMSEY)

rz_jpcm_err = limit(rz_jpcm - rz_ref)
rMSEZ = np.square(rz_jpcm_err).mean() 
rRMSEZ = math.sqrt(rMSEZ)


# print("JPCM RMSEX: %f", RMSEX)
# print("JPCM RMSEX: %f", RMSEY)
# print("JPCM RMSEX: %f", RMSEZ)

# print("JPCM rRMSEX: %f", rRMSEX)
# print("JPCM rRMSEX: %f", rRMSEY)
# print("JPCM rRMSEX: %f", rRMSEZ)

plt.rcParams.update({'font.size': 9})  # Increase font size for better readability
fig, axs = plt.subplots(2, 2, figsize=(5, 4))  # Increase figure size for better visibility

# Define a color palette
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
bottom_colors = ['#d62728', '#9467bd', '#8c564b']  # Red, purple, brown

# Function to set dense grid
def set_dense_grid(ax):
    ax.grid(True, which='major', linestyle='-', linewidth='0.5', alpha=0.7)
    ax.grid(True, which='minor', linestyle=':', linewidth='0.5', alpha=0.5)
    ax.minorticks_on()  # Enable minor ticks

# Function to set y-axis limits with a margin
def set_y_limits(ax, data_list, margin=0.1):
    data_min = min([np.min(data) for data in data_list])
    data_max = max([np.max(data) for data in data_list])
    range_margin = (data_max - data_min) * margin
    ax.set_ylim(data_min - range_margin, data_max + range_margin)

data_end = 800
# JPCM Position Errors
set_dense_grid(axs[0, 0])
axs[0, 0].plot(px_jpcm_err[0:data_end], color=colors[0], linewidth=2)
axs[0, 0].plot(py_jpcm_err[0:data_end], color=colors[1], linestyle='-.', linewidth=2)
axs[0, 0].plot(pz_jpcm_err[0:data_end], color=colors[2], linestyle=':', linewidth=2)
set_y_limits(axs[0, 0], [px_jpcm_err[0:data_end], py_jpcm_err[0:data_end], pz_jpcm_err[0:data_end]])
# axs[0, 0].legend(['x', 'y', 'z'], frameon=False, loc='upper left', bbox_to_anchor=(1, 1))
axs[0, 0].set_ylabel('Errors (m)')
axs[0, 0].set_xlabel('Time (0.01s)')
axs[0, 0].set_title('JPCM',fontsize=9)

# MPC Position Errors
set_dense_grid(axs[0, 1])
axs[0, 1].plot(px_mpc_err[0:data_end], color=colors[0], linewidth=2)
axs[0, 1].plot(py_mpc_err[0:data_end], color=colors[1], linestyle='-.', linewidth=2)
axs[0, 1].plot(pz_mpc_err[0:data_end], color=colors[2], linestyle=':', linewidth=2)
set_y_limits(axs[0, 1], [px_mpc_err[0:data_end], py_mpc_err[0:data_end], pz_mpc_err[0:data_end]])
axs[0, 1].legend(['x', 'y', 'z'], frameon=False, loc='upper left', bbox_to_anchor=(1, 1))
# axs[0, 1].set_ylabel('Errors (m)')
axs[0, 1].set_xlabel('Time (0.01s)')
axs[0, 1].set_title('MPC',fontsize=9)

# Add a shared title for the top row
fig.suptitle('Position Errors Comparison', y=0.93, fontsize=9, fontweight='bold')

# Add a shared title for the bottom row
fig.text(0.5, 0.46, 'Rotation Errors Comparison', ha='center', fontsize=9, fontweight='bold')

# JPCM Rotation Errors
set_dense_grid(axs[1, 0])
axs[1, 0].plot(rx_jpcm_err[0:data_end], color=bottom_colors[0], linewidth=2)
axs[1, 0].plot(ry_jpcm_err[0:data_end], color=bottom_colors[1], linestyle='-.', linewidth=2)
axs[1, 0].plot(rz_jpcm_err[0:data_end], color=bottom_colors[2], linestyle=':', linewidth=2)
set_y_limits(axs[1, 0], [rx_jpcm_err[0:data_end], ry_jpcm_err[0:data_end], rz_jpcm_err[0:data_end]])
# axs[1, 0].legend(['roll', 'pitch', 'yaw'], frameon=False, loc='upper left', bbox_to_anchor=(1, 1))
axs[1, 0].set_ylabel('Errors (rad)')
axs[1, 0].set_xlabel('Time (0.01s)')
axs[1, 0].set_title('JPCM',fontsize=9)

# MPC Rotation Errors
set_dense_grid(axs[1, 1])
axs[1, 1].plot(rx_mpc_err[0:data_end], color=bottom_colors[0], linewidth=2)
axs[1, 1].plot(ry_mpc_err[0:data_end], color=bottom_colors[1], linestyle='-.', linewidth=2)
axs[1, 1].plot(rz_mpc_err[0:data_end], color=bottom_colors[2], linestyle=':', linewidth=2)
set_y_limits(axs[1, 1], [rx_mpc_err[0:data_end], ry_mpc_err[0:data_end], rz_mpc_err[0:data_end]])
axs[1, 1].legend(['roll', 'pitch', 'yaw'], frameon=False, loc='upper left', bbox_to_anchor=(1, 1))
# axs[1, 1].set_ylabel('Errors (rad)')
axs[1, 1].set_xlabel('Time (0.01s)')
axs[1, 1].set_title('MPC',fontsize=9)

# plt.title("Recovery process")
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
plt.savefig('TVT_Figures/Recovery.svg', format='svg')

plt.show()


