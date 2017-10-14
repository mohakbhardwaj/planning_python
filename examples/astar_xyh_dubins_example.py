#!/usr/bin/env python
"""A minimal example that loads an environment from a png file, runs astar planner and returns the path. 
The search process and final path are rendered

Author: Mohak Bhardwaj
Date: 6 October, 2017

"""

import sys
sys.path.insert(0, "..")
import matplotlib.pyplot as plt
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.state_lattices.common_lattice.xyh_analytic_lattice import XYHAnalyticLattice
from planning_python.cost_functions.cost_function import PathLengthAng, DubinsPathLength
from planning_python.heuristic_functions.heuristic_function import DubinsHeuristic
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners.astar import Astar
import os

#Step 1: Setup some problem parameters
x_lims = [-10, 190]
y_lims = [-10, 190]
start_state = (0, 0, 0)
goal_state = (150, 150, 0)
turning_radius = 1.0
visualize = False

#Step 2: Load environment from file 
envfile = os.path.abspath("../../motion_planning_datasets/single_bugtrap/train/1.png")
env_params = {'x_lims': x_lims, 'y_lims': y_lims}
e = Env2D()
e.initialize(envfile, env_params)

#Step 3: Create lattice to overlay on environment
lattice_params = dict()
lattice_params['x_lims']          = x_lims                             # Used]ful to calculate number of cells in lattice
lattice_params['y_lims']          = y_lims                             # Useful to calculate number of cells in lattice
lattice_params['radius']          = turning_radius                     # Fixed turning radius of the robot
lattice_params['origin']          = (start_state[0], start_state[1])   # Used for conversion from discrete to continuous and vice-versa. 
lattice_params['rotation']        = 0                                  # Used for conversion from discrete to continuous and vice-versa 
lattice_params['connectivity']    = 'dubins_turn_90'                   #Lattice connectivity
lattice_params['path_resolution'] = turning_radius/2                   #Resolution for defining edges and doing collision checking (in meters)

l = XYHAnalyticLattice(lattice_params)

#Step 4: Create cost and heuristic objects
cost_fn = DubinsPathLength(turning_radius)                   #Penalize length of path
heuristic_fn = DubinsHeuristic(turning_radius)      

#(Additionally, you can precalculate edges and costs on lattice for speed-ups)
l.precalc_costs(cost_fn)						#especially helpful when lattice remains same across problems

#Step 5: Create a planning problem
prob_params = {'heuristic_weight': 1}        #Planner is not greedy at all
start_n = l.state_to_node(start_state)
goal_n = l.state_to_node(goal_state)
prob = PlanningProblem(prob_params)
prob.initialize(e, l, cost_fn, heuristic_fn, start_n, goal_n, visualize=visualize)

#Step 5: Create Planner object and ask it to solve the planning problem
planner = Astar()
planner.initialize(prob)
plan_start_time = time.time()
path, path_cost, num_expansions, came_from, cost_so_far, c_obs = planner.plan()
planning_time = time.time() - plan_start_time

print('Path: ', path)
print('Path Cost: ', path_cost)
print('Number of Expansions: ', num_expansions)
print('Time taken: ', planning_time)

e.initialize_plot(start_state, goal_state)
e.plot_edge(path)
plt.show()
