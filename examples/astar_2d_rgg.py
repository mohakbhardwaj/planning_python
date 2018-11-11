#!/usr/bin/env python
"""Astar planner running on a random geometric graph
Author: Mohak Bhardwaj 
Date: Nov 10, 2018
"""

import sys
sys.path.insert(0, "..")
import matplotlib.pyplot as plt
import numpy as np
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.graphs.rgg import RGG
from planning_python.cost_functions.cost_function import PathLengthNoAng, UnitCost
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners.astar import Astar
import os


#Step1: Set some problem parameters
x_lims = [0, 201] # low(inclusive), upper(exclusive) extents of world in x-axis
y_lims = [0, 201] # low(inclusive), upper(exclusive) extents of world in y-axis
start = (10, 10)    #start state(world coordinates)
goal = (199, 199)  #goal state(world coordinates)
visualize = False

#Step 2: Load environment from file 
envfile = os.path.abspath("../../motion_planning_datasets/forest/train/259.png")
env_params = {'x_lims': x_lims, 'y_lims': y_lims}
e = Env2D()
e.initialize(envfile, env_params)

#Step 3: Create a random geometric graph with uniformly distributed nodes
nnodes = 200
radius = 50.0
pos = {0: start, 1:goal } #Pos defines the position of the nodes in the graph. we first add the start and the goal
for i in xrange(2, nnodes): pos[i] = (np.random.uniform(x_lims[0], x_lims[1]), np.random.uniform(y_lims[0], y_lims[1]))

graph_params = dict()
graph_params['lower_limits']  = [x_lims[0], y_lims[0]]  
graph_params['upper_limits']  = [x_lims[1], y_lims[1]] 
graph_params['ndims']         = 2       
graph_params['nnodes']        = nnodes        
graph_params['radius']        = radius     
graph_params['path_resolution'] = 1         
graph_params['pos'] = pos

g = RGG(graph_params)
#Step 4: Create cost and heuristic objects
cost_fn = PathLengthNoAng()                        #Penalize length of path
heuristic_fn = EuclideanHeuristicNoAng()      

#Step 5: Create a planning problem
prob_params = {'heuristic_weight': 1.0}        
prob = PlanningProblem(prob_params)
prob.initialize(e, g, cost_fn, heuristic_fn, 0, 1, visualize=visualize)

#Step 6: Create Planner object and ask it to solve the planning problem
planner = Astar()
planner.initialize(prob)
path, path_cost, num_expansions, plan_time, came_from, cost_so_far, c_obs = planner.plan()


print('Path: ', path)
print('Path Cost: ', path_cost)
print('Number of Expansions: ', num_expansions)
print('Time taken: ', plan_time)

e.initialize_plot(start, goal, grid_res=1, plot_grid=False)
e.plot_path(path, 'solid', 'blue', 3)
e.plot_graph(g.graph, pos, arrows=False, node_size=10, edge_color='k')
plt.show()




