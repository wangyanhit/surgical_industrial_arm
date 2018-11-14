import modern_robotics as mr
import numpy as np
from math import pi
import copy
import matplotlib.pyplot as plt



'''
Xstart = [[ -1.00000000e+00,   0.00000000e+00,  -6.68545708e-07,
          3.02000337e-01],
       [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00,
          4.71350545e-08],
       [  6.68545708e-07,   0.00000000e+00,  -1.00000000e+00,
          5.57998999e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]]
Xend = copy.copy(Xstart)
Xend[0][3] += 0.1
Xend[1][3] += 0.1
Tf = 3
N = 500
method = 5

traj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
print(traj)
'''


thetastart = [1, 0, 0, 1, 1, 0.2, 0,1]
thetaend = [1.5, 0.5, 0.6, 1.1, 2, 2, 0.9, 1]
Tf = 4.0
N = 100
method = 5
traj = mr.JointTrajectory(thetastart,thetaend,Tf,N,method)
#print(traj)
#traj = np.array(traj)


x = np.linspace(0, Tf, N)
d_t = Tf/(N-1)
positions = []
positions_sum = []
velocities = []
accelerations = []
for i in range(len(traj)):
    p = [0] * 6
    v = [0] * 6
    last_v = [0] * 6
    a = [0] * 6
    if i != len(traj) - 1 and i != 0:
        for j in range(6):
            p[j] = traj[i][j]
            v[j] = (traj[i + 1][j] - traj[i][j]) /d_t
            last_v[j] = (traj[i][j] - traj[i - 1][j]) /d_t
            a[j] = (v[j] - last_v[j]) /d_t
        p_sum = positions_sum[i-1] + d_t * v[j-1]
    if i == len(traj) - 1:
        p = traj[-1]
        p_sum = positions_sum[i-1] + d_t * v[j-1]
    elif i == 0:
        p = traj[0]
        p_sum = 0.0
    positions.append(p[0])
    velocities.append(v[0])
    accelerations.append(a[0])
    positions_sum.append(p_sum)



print(positions)

plt.plot(x, positions, label='position')
plt.plot(x, positions_sum, label='position_sum')
plt.plot(x, velocities, label='velocity')
plt.plot(x, accelerations, label='acceleration')

plt.xlabel('x label')
plt.ylabel('y label')

plt.title("Simple Plot")

plt.legend()

plt.show()