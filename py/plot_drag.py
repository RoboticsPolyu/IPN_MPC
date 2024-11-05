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


def limit(x):
  for iter in range(len(x)):
    if x[iter] > 3.14159:
      x[iter] = 3.14159*2-x[iter]
    if x[iter] < -3.14158:
      x[iter] = 3.14159*2 + x[iter]
  return x


# POS_SIGMA 0.01m MPC Precise
# file = '/Users/ypwen/IPN/IPN_MPC/data/JEC_MPC_PRE_TEST_log.txt'
file = '/Users/ypwen/IPN/IPN_MPC/data/log/JPC_JPCM_TEST_Drag_log.txt'

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
file = '/Users/ypwen/IPN/IPN_MPC/data/log/JPC_JPCM_TEST_NoDrag_0.1_log.txt'

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
    ax.ylim(data_min - range_margin, data_max + range_margin)

plt.figure(figsize=(4, 3.7))
set_dense_grid(plt)
plt.plot(px_mpc[0:1000], py_mpc[0:1000], color=colors[0], linestyle='-.', linewidth =1.5)
plt.plot(px_jpcm[0:1000], py_jpcm[0:1000],  color=colors[1], linestyle='-', linewidth =1.5)
plt.xlabel('X-axis (m)') #注意后面的字体属性
plt.ylabel('Y-axis (m)')
plt.legend(['JPCM-Drag', 'JPCM'], loc='upper right', bbox_to_anchor=(1.15, 1), frameon=False)
plt.title('Path comparison of JPCM and JPCM-Drag')  
plt.savefig('TVT_Figures/JPCM-Drag.svg',format='svg')

plt.show()



