import numpy as np
import matplotlib.pyplot as plt
plt.ion()
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('map', help='Enter the map you want to modify among these two : \n 1. Map1 ---> 1 \n 2. Map7 ---> 7', type=int)
args = parser.parse_args()
map_no = args.map
assert isinstance(map_no, int) and map_no in [1,7]


envmap = np.loadtxt(f'./maps/map{map_no}.txt')
x_size,y_size = envmap.shape

res = 10 if map_no == 7 else 5
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

np.savetxt(f"./maps/map{map_no}_modified.txt", newenvmap,fmt = '%d' )