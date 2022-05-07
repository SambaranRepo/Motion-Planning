from urllib import robotparser
import numpy as np
import math
import time
from pqdict import pqdict
from targetplanner import targetplanner

def robotplanner(envmap, robotpos, targetpos, state_space):
  # all possible directions of the robot
  numofdirs = 8
  dX = [-1, -1, -1, 0, 0, 1, 1, 1]
  dY = [-1,  0,  1, -1, 1, -1, 0, 1]
  path = []
  
  # use the old position if we fail to find an acceptable move
  # newrobotpos = np.copy(robotpos)
  
  # # for now greedily move towards the target 
  # # but this is the gateway function for your planner 
  # mindisttotarget = 1000000
  # for dd in range(numofdirs):
  #   newx = robotpos[0] + dX[dd]
  #   newy = robotpos[1] + dY[dd]
  
  #   if (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1]):
  #     if(envmap[newx, newy] == 0):
  #       disttotarget = math.sqrt((newx-targetpos[0])**2 + (newy-targetpos[1])**2)
  #       if(disttotarget < mindisttotarget):
  #         mindisttotarget = disttotarget
  #         newrobotpos[0] = newx
  #         newrobotpos[1] = newy
  
graph = {}

class Node():
  def __init__(self, envmap, targetpos):
    self.x_max = envmap.shape[0]
    self.y_max = envmap.shape[1]
    self.target = targetpos
  
  def node_id(self, x, y):
    self.id = self.y_max * x + y
  
  def heuristic(self,x,y):
    self.h = np.linalg.norm([x, y]- self.target, 1)
  
  def label(self):
    self.g = np.inf
  
  def return_attribs(self,x,y):
    self.node_id(x,y)
    self.heuristic(x,y)
    self.label()
    return {self.id: {'pos': (x, y), 'g': self.g, 'h':self.h}}


class Environment():
  def __init__(self, envmap, target_pos):
    self.map = envmap
    self.target_pos = target_pos
    self.numofdirs = 8
    self.dX = [-1, -1, -1, 0, 0, 1, 1, 1]
    self.dY = [-1,  0,  1, -1, 1, -1, 0, 1]
    self.node = Node(envmap, target_pos)
    self.x_max = self.map.shape[0]
    self.y_max = self.map.shape[1]

  def isGoal(self, node):
    if all(self.graph[node]['pos'] == self.target_pos):
      return True
    else:
      return False

  def getSuccessors(self, node):
    current_pos = self.node.return_attribs(node // self.y_max, node % self.y_max)[node]['pos']
    successor_nodes = []
    costs = []
    action_id = []
    for i in range(self.numofdirs):
      x_new = (current_pos[0] + self.dX[i])
      y_new = (current_pos[1] + self.dY[i])
      # print(f'x new : {x_new}')
      # print(f' y new : {y_new}')
      if x_new < 0 or y_new < 0 or x_new >= self.map.shape[0] or y_new >= self.map.shape[1]:
        continue
      else:
        if self.map[x_new, y_new] == 1 :
          cost = np.inf
        else:
          cost = np.linalg.norm([self.dX[i], self.dY[i]])
        # print(f'graph : {self.graph}')
        child = self.y_max * x_new + y_new

        successor_nodes.append(child)
        costs.append(cost)
        action_id.append(i)
    return successor_nodes, costs, action_id

  def getHeuristic(self, node, target_pos):
    # print(f'node : {node}')
    # print(f"position : {self.graph[node]['pos']}")
    # print(f'target pos : {target_pos}')
    pos = self.node.return_attribs(node // self.y_max, node % self.y_max)[node]['pos']
    return np.linalg.norm(pos - target_pos)

class RTAA():
  def __init__(self, robotpos, targetpos, env, envmap):
    self.env = env
    self.node = Node(envmap, targetpos)
    self.x_max = envmap.shape[0]
    self.y_max = envmap.shape[1]
    self.target_pos = targetpos
    self.target_id = self.y_max * targetpos[0] + targetpos[1]
    self.robot_pos = robotpos
    self.robot_id = self.y_max * robotpos[0] + robotpos[1]
    self.open = pqdict()
    self.close = []
    self.parent = {}
    graph.update(self.node.return_attribs(*self.robot_pos))
    graph[self.robot_id]['g'] = 0

  def rtaa(self):
    robot_id = self.robot_id
    self.open[robot_id] = graph[robot_id]['g'] + graph[robot_id]['h']
    max_nodes = 500
    expanded = 0
    caught = False
    while expanded < max_nodes:
      popped_id = self.open.pop()
      successors, costs, action = self.env.getSuccessors(popped_id)
      self.close.append(popped_id)
      if self.target_id in self.close:
        caught = True
        break
      else:
        for i in range(len(successors)):
          if successors[i] not in graph:
            self.child = self.node.return_attribs(successors[i] // self.y_max, successors[i] % self.y_max)
            graph.update(self.child)
          

          if graph[successors[i]]['g'] > graph[popped_id]['g'] + costs[i]:
            graph[successors[i]]['g'] = graph[popped_id]['g'] + costs[i]
            if successors[i] in self.open:
              self.open[successors[i]] = graph[successors[i]]['g'] + self.env.getHeuristic(successors[i], self.target_pos)
              # self.open[successors[i]] = self.graph[successors[i]]['g'] + self.graph[successors[i]]['h']
              self.parent[successors[i]] = popped_id
            elif successors[i] not in self.open and successors[i] not in self.close:
              self.open.update({successors[i] : graph[successors[i]]['g'] + self.env.getHeuristic(successors[i], self.target_pos)})
              # self.open.update({successors[i] : self.graph[successors[i]]['g'] +self.graph[successors[i]]['h']})
              self.parent.update({successors[i] : popped_id})
        
      expanded = len(self.close)


    
    
    for key in self.open:
      graph[key]['h'] = self.env.getHeuristic(key, self.target_pos)

    if not caught:
      best_open_node, f_star = self.open.popitem()
    else:
      best_open_node, f_star = self.target_id, graph[self.target_id]['g'] + graph[self.target_id]['h'] 
      for key in self.close:
        graph[key]['h'] = f_star - graph[key]['g']
    child = best_open_node
    par = 0
    while True:
      par = self.parent[child]
      if par == robot_id:
        break
      else:
        child = par

    next_node = child
  
    
    self.next = (next_node // self.y_max, next_node % self.y_max)
    robotnextpos = graph[next_node]['pos']
    print(f'robot next pos : {robotnextpos[0], robotnextpos[1]}')
    return robotnextpos

  




