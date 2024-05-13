import matplotlib.pyplot as plt
from scipy import interpolate
import numpy as np

from matplotlib import cm
from mpl_toolkits.mplot3d import axes3d

from matplotlib import animation



font = {'family' : 'Times New Roman',
        #'weight' : 'bold',
        'size'   : 10.5}

plt.rc('font', **font)

file = '../data/Traj/Traj_mahua.txt'

data = np.loadtxt(file)

time = data[:,0] 
p_x = data[:,1] 
p_y = data[:,2] 
p_z = data[:,3]
q_w = data[:,4]
q_x = data[:,5]
q_y = data[:,6]
q_z = data[:,7]
v_x = data[:,8]
v_y = data[:,9]
v_z = data[:,10]

plt.figure(1)
plt.plot(time - time[0], p_x, '-r', time- time[0], p_y, '-b', time- time[0], p_z, '-g')
plt.legend(['p_x', 'p_y', 'p_z'])
plt.xlabel('Time') #注意后面的字体属性
plt.ylabel('Position')
plt.title('Position')  


plt.figure(2)
plt.plot(time - time[0], v_x, '-r', time- time[0], v_y, '-b', time- time[0], v_z, '-g')
plt.legend(['v_x', 'v_y', 'v_z'])
plt.xlabel('Time') #注意后面的字体属性
plt.ylabel('Velocity')
plt.title('Velocity')  


# plt.savefig('PWM.jpg')
ax = plt.figure().add_subplot(projection='3d')

line = ax.plot(p_x, p_y, p_z, label='type curve')
ax.plot(p_x[0:1], p_y[0:1], p_z[0:1], 'ro')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')     
plt.title('Position')   


plt.show()


