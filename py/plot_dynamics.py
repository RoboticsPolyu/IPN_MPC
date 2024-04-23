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

dyn_vel_err_x = data[:,4] 
dyn_vel_err_y = data[:,5] 
dyn_vel_err_z = data[:,6]

thrust_ex = data[:,13]
thrust_ey = data[:,14]
thrust_ez = data[:,15]

tq_ex = data[:,16]
tq_ey = data[:,17]
tq_ez = data[:,18]

df_ex = data[:,19]
df_ey = data[:,20]
df_ez = data[:,21]


plt.figure(1)
plt.plot((timestamp - timestamp[0])/1e9, df_ex, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0])/1e9, df_ey, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0])/1e9, thrust_ez, color = 'green', linestyle='-')
plt.plot((timestamp - timestamp[0])/1e9, df_ez, color = 'purple', linestyle='-')

plt.legend(['Predicted body-x drag force', 'Predicted body-y drag force', 'Predicted thrust', 'Predicted body-z drag force'])
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.title('Predicted drag force and thrust')  

plt.figure(2)
plt.plot((timestamp - timestamp[0])/1e9, dyn_vel_err_x*100, color = 'darkred', linestyle='-')
plt.plot((timestamp - timestamp[0])/1e9, dyn_vel_err_y*100, color = 'darkblue', linestyle='-')
plt.plot((timestamp - timestamp[0])/1e9, dyn_vel_err_z*100, color = 'purple', linestyle='-')
plt.xlabel('Time (s)') #注意后面的字体属性
plt.ylabel('Force (N)')
plt.title('Predicted force error')  

plt.show()