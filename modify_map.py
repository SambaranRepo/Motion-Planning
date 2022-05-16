import numpy as np
import matplotlib.pyplot as plt
plt.ion()


envmap = np.loadtxt('./maps/map7.txt')
x_size,y_size = envmap.shape

res = 10
newenvmap = np.zeros((x_size // res + 1, y_size //res + 1))
for i in range(0, x_size, res):
    for j in range(0, y_size, res):
        if envmap[i:i+res, j:j+res].any() == 1:
            newenvmap[i//res,j//res] = 1
        

f, ax = plt.subplots()
ax.imshow( newenvmap.T, interpolation="none", cmap='gray_r', origin='lower', \
            extent=(-0.5, newenvmap.shape[0]-0.5, -0.5, newenvmap.shape[1]-0.5) )
ax.axis([-0.5, newenvmap.shape[0]-0.5, -0.5, newenvmap.shape[1]-0.5])
ax.set_xlabel('x')
ax.set_ylabel('y')  
plt.show(block = True)

np.savetxt("./maps/map7_modified.txt", newenvmap,fmt = '%d' )