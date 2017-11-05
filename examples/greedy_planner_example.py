#!/usr/bin/env python
"""A minimal example that loads an environment from a png file, runs greedy planner and returns the path. 
The search process and final path are rendered

Author: Mohak Bhardwaj
Date: 28 October, 2017

"""

import sys
sys.path.insert(0, "..")
import matplotlib.pyplot as plt
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.state_lattices.common_lattice.xy_analytic_lattice import XYAnalyticLattice
from planning_python.cost_functions.cost_function import PathLengthNoAng, UnitCost
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners.greedy_planner import GreedyPlanner
import os


#Step1: Set some problem parameters
x_lims = [0, 200] # low(inclusive), upper(exclusive) extents of world in x-axis
y_lime = [0, 200] # low(inclusive), upper(exclusive) extents of world in y-axis
start = (0, 0)    #start state(world coordinates)
goal = (199,199)  #goal state(world coordinates)
visualize = True

#Step 2: Load environment from file 
envfile = os.path.abspath("../../motion_planning_datasets/single_bugtrap/train/1.png")
env_params = {'x_lims': [0, 200], 'y_lims': [0, 200]}
e = Env2D()
e.initialize(envfile, env_params)

#Step 3: Create lattice to overlay on environment
lattice_params = dict()
lattice_params['x_lims']          = [0, 200] # Usefule to calculate number of cells in lattice 
lattice_params['y_lims']          = [0, 200] # Useful to calculate number of cells in lattice
lattice_params['resolution']      = [1, 1]   # Useful to calculate number of cells in lattice + conversion from discrete to continuous space and vice-versa
lattice_params['origin']          = start    # Used for conversion from discrete to continuous and vice-versa. 
lattice_params['rotation']        = 0        # Can rotate lattice with respect to world
lattice_params['connectivity']    = 'eight_connected' #Lattice connectivity (can be four or eight connected for xylattice)
lattice_params['path_resolution'] = 1         #Resolution for defining edges and doing collision checking (in meters)

l = XYAnalyticLattice(lattice_params)

#Step 4: Create cost and heuristic objects
cost_fn = PathLengthNoAng()                        #Penalize length of path
heuristic_fn = EuclideanHeuristicNoAng()      

#(Additionally, you can precalculate edges and costs on lattice for speed-ups)
l.precalc_costs(cost_fn)						#useful when lattice remains same across problems
#Step 5: Create a planning problem
prob_params = {'heuristic_weight': 1.0}        
start_n = l.state_to_node(start)
goal_n = l.state_to_node(goal)
prob = PlanningProblem(prob_params)
prob.initialize(e, l, cost_fn, heuristic_fn, start_n, goal_n, visualize=visualize)

#Step 6: Create Planner object and ask it to solve the planning problem
planner = GreedyPlanner()
planner.initialize(prob)
path, path_cost, num_expansions, plan_time, came_from, cost_so_far, c_obs = planner.plan()


print('Path: ', path)
print('Path Cost: ', path_cost)
print('Number of Expansions: ', num_expansions)
print('Time taken: ', plan_time)

e.initialize_plot(start, goal)
e.plot_path(path, 'solid', 'red', 3)
plt.show()




