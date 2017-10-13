#!/usr/bin/env python
import sys
sys.path.insert(0, "../..")
import matplotlib.pyplot as plt
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.state_lattices.common_lattice.xy_analytic_lattice import XYAnalyticLattice
from planning_python.cost_functions.cost_function import PathLengthNoAng
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners.astar import Astar
import os

#Step 1: Load environment from file 
envfile = os.path.abspath("../../databases/bugtrap.png")
env_params = {'x_lims': [0, 100], 'y_lims': [0,100]}
e = Env2D()
e.initialize(envfile, env_params)

#Step 2: Create lattice to overlay on environment
lattice_params = dict()
lattice_params['x_lims']          = [0, 100] # Used to calculate number of cells in lattice (should ideally be consistent with env_params['x_lims'])
lattice_params['y_lims']          = [0, 100] # Used to calculate number of cells in lattice (should ideally be consistent with env_params['y_lims'])
lattice_params['resolution']      = [1, 1]   # Used to calculate number of cells in lattice + conversion from discrete to continuous space and vice-versa
lattice_params['origin']          = (0, 0)   # Used for conversion from discrete to continuous and vice-versa. 
lattice_params['rotation']        = 0        # Used for conversion from discrete to continuous and vice-versa (This plus origin define lattice-->world transform)
lattice_params['connectivity']    = 'eight_connected' #Lattice connectivity (can be four or eight connected for xylattice)
lattice_params['path_resolution'] = 1      #Resolution for defining edges and doing collision checking (in meters)

l = XYAnalyticLattice(lattice_params)

#Step 3: Create cost and heuristic objects
cost_fn = PathLengthNoAng()                   #Penalize length of path
heuristic_fn = EuclideanHeuristicNoAng()      

#Step 4: Create a planning problem
prob_params = {'heuristic_weight': 1}        #Planner is not greedy at all
start_n = l.state_to_node((0,0))
goal_n = l.state_to_node((100, 100))
prob = PlanningProblem(e, l, cost_fn, heuristic_fn, prob_params, start_n, goal_n, visualize=True)

#Step 4: Create Planner object and ask it to solve the planning problem
planner = Astar(prob)
plan_start_time = time.time()
path, path_cost, num_expansions, came_from, cost_so_far, c_obs = planner.plan()
planning_time = time.time() - plan_start_time
print('Path: ', path)
print('Path Cost: ', path_cost)
print('Number of Expansions: ', num_expansions)
print('Time taken: ', planning_time)

plt.show()



