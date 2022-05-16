from distutils.dep_util import newer_group
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
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('planner', help='Which Motion Planning model would you like to implement : \n 1. Agent Centered Search ---> rtaa \n \
2. Anytime Search ---> ara')
args = parser.parse_args()
algorithm = args.planner
assert isinstance(algorithm, str)

# functions to time how long planning takes  
def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))


def runtest(envmap, robotstart, targetstart, map):
  # current positions of the target and robot
  robotpos = np.copy(robotstart);
  targetpos = np.copy(targetstart);
  env = Environment(envmap, targetpos)
  folder = './images/'
  # environment
  # envmap = loadtxt(mapfile)
    
  # draw the environment
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
  for i in tqdm(range(20000)):
    # call robot planner
    if algorithm == 'rtaa':
      planner = RTAA(robotpos, targetpos,env, envmap)
    else:
       planner = AnytimeA_star(robotpos, targetpos,env, envmap)
    type_planner = 'AgentCentred' if isinstance(planner, RTAA) else 'Anytime'
    t0 = tic()

    # newrobotpos = robotplanner(envmap, robotpos, targetpos, state_space)
    newrobotpos = planner.plan()
    print(f'new robot pos : {newrobotpos}')
    robot_trajectory.append(newrobotpos)
    # compute move time for the target, if it is greater than 2 sec, the target will move multiple steps
    print(f'planning time was : {tic() - t0}')
    movetime = max(1, math.ceil((tic()-t0)/2.0))
    # print(f'target moves : {movetime} steps')
    #check that the new commanded position is valid
    if ( newrobotpos[0] < 0 or newrobotpos[0] >= envmap.shape[0] or \
         newrobotpos[1] < 0 or newrobotpos[1] >= envmap.shape[1] ):
      print('ERROR: out-of-map robot position commanded\n')
      break
    elif ( envmap[newrobotpos[0], newrobotpos[1]] != 0 ):
      print('ERROR: invalid robot position commanded\n')
      break
    elif (abs(newrobotpos[0]-robotpos[0]) > 1 or abs(newrobotpos[1]-robotpos[1]) > 1):
      print('ERROR: invalid robot move commanded\n')
      break

    # call target planner to see how the target moves within the robot planning time
    newtargetpos = targetplanner(envmap, robotpos, targetpos, targetstart, movetime)
    target_trajectory.append(newtargetpos)
    # make the moves
    robotpos = newrobotpos
    targetpos = newtargetpos
    numofmoves += 1
    print(f'robot position : {robotpos}')
    print(f'target pos : {targetpos}')
    
    # draw positions
    hr[0].set_xdata(robotpos[0])
    hr[0].set_ydata(robotpos[1])
    ht[0].set_xdata(targetpos[0])
    ht[0].set_ydata(targetpos[1])
    f.canvas.flush_events()
    plt.show()

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
      plt.savefig(folder + map + f'_{type_planner}.png', format = 'png', bbox_inches = 'tight')
      break

      

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
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map1')

def test_map2():
  robotstart = np.array([0, 2])
  targetstart = np.array([7, 9])
  mapfile = 'maps/map2.txt'
  envmap = loadtxt(mapfile)
 # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map2')
  
def test_map3():
  robotstart = np.array([249, 249])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  # return runtest(envmap, robotstart, targetstart, state_space)
  return runtest(envmap, robotstart, targetstart, 'map3')

def test_map4():
  robotstart = np.array([0, 0])
  targetstart = np.array([5, 6])
  mapfile = 'maps/map4.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map4')

def test_map5():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 59])
  mapfile = 'maps/map5.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map5')

def test_map6():
  robotstart = np.array([0, 0])
  targetstart = np.array([29, 36])
  mapfile = 'maps/map6.txt'
  envmap = loadtxt(mapfile)
 # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map6')

def test_map7():
  robotstart = np.array([0//10, 0//10])
  targetstart = np.array([4998//10, 4998//10])
  mapfile = 'maps/map7_modified.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map7')


def test_map1b():
  robotstart = np.array([249//5, 1199//5])
  targetstart = np.array([1649//5, 1899//5])
  mapfile = 'maps/map1_modified.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map1b')

def test_map3b():
  robotstart = np.array([74, 249])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map3b')

def test_map3c():
  robotstart = np.array([4, 399])
  targetstart = np.array([399, 399])
  mapfile = 'maps/map3.txt'
  envmap = loadtxt(mapfile)
  # state_space = StateSpace(envmap)
  # state_space.create_hash()
  # state_space.graph[state_space.reverse_graph[tuple(robotstart)]]['g'] = 0
  return runtest(envmap, robotstart, targetstart, 'map3c')

if __name__ == "__main__":
  # you should change the following line to test different maps
  caught, numofmoves = test_map1b()
  print('Number of moves made: {}; Target caught: {}.\n'.format(numofmoves, caught))
  plt.ioff()
  plt.show()



