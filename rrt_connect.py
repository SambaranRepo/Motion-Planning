'''
: This script tries to run the RRTConnect algorithm on the maps.
'''


import numpy as np
import math
from numpy import isin, loadtxt
import matplotlib.pyplot as plt
plt.ion()
import time
from pqdict import minpq
from robotplanner import *
from targetplanner import targetplanner
from tqdm import tqdm
from src.rrt.rrt_connect import RRTConnect
from src.search_space.search_space import SearchSpace
import os

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


def runtest(envmap, robotstart, targetstart, map, map_7 = False):
  # current positions of the target and robot
  robotpos = np.copy(robotstart);
  targetpos = np.copy(targetstart);
  env = Environment(envmap, targetpos)
  folder = './images/'

  # transpose because imshow places the first dimension on the y-axis
  f, ax = plt.subplots()
  ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
             extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
  ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
  ax.set_xlabel('x')
  ax.set_ylabel('y')  
  hr = ax.plot(robotpos[0], robotpos[1], 'bs')
  ht = ax.plot(targetpos[0], targetpos[1], 'rs')
  f.canvas.flush_events()
  robot_trajectory = []
  target_trajectory = []
  robot_trajectory.append(tuple(robotstart))
  target_trajectory.append(tuple(targetstart))
  # now comes the main loop
  numofmoves = 0
  caught = False
  movetime_rrt = 0
  for i in tqdm(range(20000)):
    # call robot planner
    X_dimensions = np.array([(0, envmap.shape[0]-1), (0, envmap.shape[1]-1)])  # dimensions of Search Space
    max_samples = 50000
    prc = 0.1
    rewire_count = 64
    Q = [(1,1)]
    r = 0.2  # length of smallest edge to check for intersection with obstacles
    t0 = time.time()
    if i%600 == 0 or index == len(path)-1:
    # if i == 0 or index > len(path) - 2:
          index = 1
          X = SearchSpace(X_dimensions, envmap, None)
          planner = RRTConnect(X, Q, tuple(robotpos), tuple(targetpos), max_samples, r, prc)
          path = planner.plan()
          newrobotpos = (int(np.around(path[index][0])), int(np.around(path[index][1])))
          if newrobotpos[0] == robotpos[0] or newrobotpos[1] == robotpos[1]:
            movetime = int(np.linalg.norm(np.array(newrobotpos) - np.array(robotpos), 1)) + int((time.time() - t0)/2)
          else:
            movetime = int(np.linalg.norm(np.array(newrobotpos) - np.array(robotpos),2) / math.sqrt(2)) + int((time.time() - t0)/2)
    else:
        index += 1
        newrobotpos = (int(np.around(path[index][0])), int(np.around(path[index][1])))
        if newrobotpos[0] == robotpos[0] or newrobotpos[1] == robotpos[1]:
            movetime = int(np.linalg.norm(np.array(newrobotpos) - np.array(robotpos), 1))
        else:
            movetime = int(np.linalg.norm(np.array(newrobotpos) - np.array(robotpos),2) / math.sqrt(2))
    print(f'movetime : {movetime}')
    movetime_rrt +=movetime

    robot_trajectory.append(newrobotpos)
    
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
          newrobotpos[1] < 0 or newrobotpos[1] >=envmap.shape[1] ):
        print('ERROR: out-of-map robot position commanded\n')
        break
    elif (envmap[newrobotpos[0], newrobotpos[1]] != 0):
        print('ERROR: invalid robot position commanded\n')
        break
    
    newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime)
    target_trajectory.append(newtargetpos)
    # make the moves
    robotpos = newrobotpos
    targetpos = newtargetpos
    numofmoves += 1
    
    print(f'target pos : {targetpos}')
    
    # draw positions
    hr[0].set_xdata(robotpos[0])
    hr[0].set_ydata(robotpos[1])
    ht[0].set_xdata(targetpos[0])
    ht[0].set_ydata(targetpos[1])
    f.canvas.flush_events()
    plt.show()
    if 'rrt_connect' not in os.listdir(f'./animations/{map}'):
        os.mkdir(f'./animations/{map}/rrt_connect')
    plt.savefig(f'./animations/{map}/rrt_connect/{i}.png', format = 'png', bbox_inches = 'tight')

    # check if target is caught
    if (abs(robotpos[0]-targetpos[0]) <= 1 and abs(robotpos[1]-targetpos[1]) <= 1):
      print('robotpos = (%d,%d)' %(robotpos[0],robotpos[1]))
      print('targetpos = (%d,%d)' %(targetpos[0],targetpos[1]))
      robot_trajectory.append(targetpos)
      x = [item[0] for item in robot_trajectory]
      y= [item[1] for item in robot_trajectory]
      target_x = [item[0] for item in target_trajectory]
      target_y = [item[1] for item in target_trajectory]
      caught = True
      plt.close()
      f, ax = plt.subplots()
      ax.imshow( envmap.T, interpolation="none", cmap='gray_r', origin='lower', \
                extent=(-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5) )
      ax.axis([-0.5, envmap.shape[0]-0.5, -0.5, envmap.shape[1]-0.5])
      ax.set_xlabel('x')
      ax.set_ylabel('y')  
      hr = ax.plot(robotstart[0], robotstart[1], 'bs')
      ht = ax.plot(targetstart[0], targetstart[1], 'rs')
      hr = ax.plot(x, y, 'b')
      ht = ax.plot(target_x, target_y, 'r')
      hr = ax.plot(x[-1], y[-1], 'bx')
      # plt.show(block = True)
      plt.savefig(folder + map + f'_rrt_connect.png', format = 'png', bbox_inches = 'tight')
      break

    numofmoves = movetime_rrt      

  return caught, numofmoves

def test_map0():
  robotstart = np.array([0, 2])
  targetstart = np.array([5, 3])
  mapfile = 'maps/map0.txt'
  envmap = loadtxt(mapfile)
  
  return runtest(envmap, robotstart, targetstart, 'map0')

def test_map1():
  robotstart = np.array([699, 799])
  targetstart = np.array([699, 1699])
  mapfile = 'maps/map1.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map1')

def test_map2():
  robotstart = np.array([0, 2])
  targetstart = np.array([7, 9])
  mapfile = 'maps/map2.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map2')
  
def test_map3():
  robotstart = np.array([249, 249])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map3')

def test_map4():
  robotstart = np.array([0, 0])
  targetstart = np.array([5, 6])
  mapfile = 'maps/map4.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map4')

def test_map5():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 59])
  mapfile = 'maps/map5.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map5')

def test_map6():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 36])
  mapfile = 'maps/map6.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map6')

def test_map7():
  robotstart = np.array([0, 0])
  targetstart = np.array([int(0.1 * 4998), int(0.1 *4998)])
  # targetstart = np.array([int(4998), int(4998)])
  mapfile = 'maps/map7_modified.txt'
  # mapfile = 'maps/map7.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map7', map_7=True)


def test_map1b():
  robotstart = np.array([249//5, 1199//5])
  targetstart = np.array([1649//5, 1899//5])
  mapfile = 'maps/map1_modified.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map1b')

def test_map3b():
  robotstart = np.array([74, 249])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map3b')

def test_map3c():
  robotstart = np.array([4, 399])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  return runtest(envmap, robotstart, targetstart, 'map3c')

if __name__ == "__main__":
  # you should change the following line to test different maps
  caught, numofmoves = test_map1b()
  print('Number of moves made: {}; Target caught: {}.\n'.format(numofmoves, caught))
  plt.ioff()
  plt.show()