import numpy as np

from src.rrt.rrt_star import RRTStar
from src.search_space.search_space import SearchSpace
from src.utilities.plotting import Plot

envmap = np.loadtxt('./maps/map7_modified.txt')
X_dimensions = np.array([(0, envmap.shape[0]-1), (0, envmap.shape[1]-1)])  # dimensions of Search Space
# obstacles
# Obstacles = np.array([(20, 20, 40, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)])
x_init = (0, 0)  # starting location
x_goal = (4998//10, 4998//10)  # goal location

Q = np.array([(1,2)])  # length of tree edges
r = 5  # length of smallest edge to check for intersection with obstacles
max_samples = 50000  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal
rewire_count = 32
# create search space
X = SearchSpace(X_dimensions, envmap, None)

# create rrt_search
rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
path = rrt.rrt_star()
print(f'path to goal : {path}')
# plot
# plot = Plot("rrt_2d")
# plot.plot_tree(X, rrt.trees)
# if path is not None:
#     plot.plot_path(X, path)
# plot.plot_obstacles(X, Obstacles)
# plot.plot_start(X, x_init)
# plot.plot_goal(X, x_goal)
# plot.draw(auto_open=True)