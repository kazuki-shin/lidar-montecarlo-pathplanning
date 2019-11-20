import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mat = np.load("insert-file-name-here.npy")

xs = [row[0] for row in mat]
ys = [row[1] for row in mat]
zs = [row[2] for row in mat]
mat = np.asarray([xs,ys,zs]).T

# filtering - currently hardcoded
mat = mat[(mat[:,2]<1.5) & (mat[:,2]>-1.5)] # z-axis
mat = mat[(mat[:,0]>=0.5) | (mat[:,0]<=-0.5)] # x-axis
mat = mat[(mat[:,1]>=1) | (mat[:,1]<=-1)] # y-axis

# 3D visualization
fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')
ax.scatter(mat[:,0],mat[:,1],mat[:,2],c=mat[:,2])
plt.show()

# 2D for planning
grid = np.column_stack((mat[:,0],mat[:,1]))

fig = plt.figure()
plt.scatter(mat[:,0],mat[:,1])
plt.show()
