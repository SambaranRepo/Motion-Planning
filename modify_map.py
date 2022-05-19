'''
: This script resizes map1b and map7 using cv2.resize
: The new map is saved as modified_map in  the maps folder and motion planning algorithms are tested in t hese modified maps.
'''


from configparser import Interpolation
import numpy as np
import matplotlib.pyplot as plt
import cv2
plt.ion()
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('map', help='Enter the map you want to modify among these two : \n 1. Map1 ---> 1 \n 2. Map7 ---> 7', type=int)
args = parser.parse_args()
map_no = args.map
assert isinstance(map_no, int) and map_no in [1,7]


envmap = np.loadtxt(f'./maps/map{map_no}.txt')
x_size,y_size = envmap.shape
res = 0.1 if map_no == 7 else 0.2

newenvmap = cv2.resize(envmap, (int(res * envmap.shape[1]), int(res* envmap.shape[0])), interpolation = cv2.INTER_CUBIC)
f, ax = plt.subplots()
ax.imshow( newenvmap.T, interpolation="none", cmap='gray_r', origin='lower', \
            extent=(-0.5, newenvmap.shape[0]-0.5, -0.5, newenvmap.shape[1]-0.5) )
ax.axis([-0.5, newenvmap.shape[0]-0.5, -0.5, newenvmap.shape[1]-0.5])
ax.set_xlabel('x')
ax.set_ylabel('y')  
plt.show(block = True)

np.savetxt(f"./maps/map{map_no}_modified.txt", newenvmap, fmt="%d")