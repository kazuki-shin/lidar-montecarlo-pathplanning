import numpy as np
from matplotlib import pyplot as plt
data = np.load('turn_data.npy')
plt.plot(data[:,0],data[:,1])
plt.show()
