#!/usr/bin/env python
"""A minimal example that loads an environment from a png file, runs lazy shortest path planner and returns the path. 
The search process and final path are rendered

Author: Mohak Bhardwaj
Date: 20 October, 2018

Reference: http://personalrobotics.ri.cmu.edu/files/courses/16843/notes/lazysp/lazysp-2016.pdf

POLCIES:
0 - SelectExpand
1 - SelectForward
2 - SelectReverse
3 - SelectAlternate
4 - SelectBisection 
"""

import sys, os
sys.path.insert(0, os.path.abspath(".."))
import json
import matplotlib.pyplot as plt
import numpy as np
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.graphs.rgg import RGG
from planning_python.cost_functions.cost_function import PathLengthNoAng, UnitCost, LazyCost
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners import LSPPlanner, Astar
from planning_python.utils import helpers

np.random.seed(0)
NUM_POLICIES = 4 #We are going to evaluate all the naive policies defined in the paper
x_lims = [0, 200] # low(inclusive), upper(exclusive) extents of world in x-axis
y_lims = [0, 200] # low(inclusive), upper(exclusive) extents of world in y-axis
start = (0, 0)
goal = (199, 199)

env_params = {'x_lims': x_lims, 'y_lims': y_lims}
nnodes = 150
radius = 50.0
pos = {0: start, 1:goal } #Pos defines the position of the nodes in the graph. we first add the start and the goal
for i in xrange(2, nnodes): pos[i] = (np.random.uniform(x_lims[0], x_lims[1]), np.random.uniform(y_lims[0], y_lims[1]))

graph_params = dict()
graph_params['lower_limits']  = [x_lims[0], y_lims[0]]  
graph_params['upper_limits']  = [x_lims[1], y_lims[1]] 
graph_params['ndims']         = 2       
graph_params['nnodes']        = nnodes        
graph_params['radius']        = radius     
graph_params['path_resolution'] = 0.5         
graph_params['pos'] = pos

prob_params = {'heuristic_weight': 1.0}


true_cost_fn = PathLengthNoAng()  #Penalize length of path
heuristic_fn = EuclideanHeuristicNoAng()      
lazy_cost_fn = LazyCost(PathLengthNoAng(), weight=1)
planner = LSPPlanner()
base_planner = Astar()
visualize = False
suppress_output = True


def run_benchmark(database_folders=[], num_envs=1):
  global env_params, graph_params, prob_params, true_cost_fn, heuristic_fn, lazy_cost_fn, planner, base_planner, start, goal, NUM_POLICIES, visualize, suppress_output
  from collections import defaultdict
  e = Env2D()
  print('Running benchmark')

  for f, folder in enumerate(database_folders):
    results= defaultdict(list)
    print('Current folder %s'%(database_folders[f]))    
    for i in range(num_envs):
      #Initialize the environment
      curr_env_file = os.path.join(os.path.abspath(folder), str(i)+'.png')
      print('Curr env number = %d'%i)
      e.initialize(curr_env_file, env_params)
      #Define random geometric graph      
      lower_limits = graph_params['lower_limits']
      upper_limits = graph_params['upper_limits']
      pos = {0: start, 1:goal } #Pos defines the position of the nodes in the graph. we first add the start and the goal
      for i in xrange(2, nnodes): pos[i] = (np.random.uniform(lower_limits[0], upper_limits[0]), np.random.uniform(lower_limits[1], upper_limits[1]))      
      graph_params['pos'] = pos
      graph = RGG(graph_params)

      for policy in range(NUM_POLICIES):
        print('Curr policy number = %d'%policy) 
        prob = PlanningProblem(prob_params)
        prob.initialize(e, graph, true_cost_fn, heuristic_fn, start_n=0, goal_n=1, visualize=visualize)
        prob.set_lazy_cost(lazy_cost_fn)
        planner.initialize(prob, base_planner, policy=policy)
        print('Calling LSP planner') 
        if suppress_output:
          with helpers.nostdout():
            path, path_cost, num_edge_evals, plan_time, num_iters, num_base_calls = planner.plan(max_iters=np.inf, suppress_base_output=True)
        else:
          path, path_cost, num_edge_evals, plan_time, num_iters, num_base_calls = planner.plan(max_iters=np.inf, suppress_base_output=True)
        results[policy].append((path_cost, num_edge_evals, num_iters, num_base_calls, plan_time))

        print('Path cost: ', path_cost)
        print('Num edge evaluations: ', num_edge_evals)
        print('Time taken: ', plan_time)
        print('Num planning iterations', num_iters)
        print('Num base planner calls', num_base_calls)
        planner.reset() #clear the planner at the end
    
    env_name = os.path.split(os.path.split(os.path.abspath(folder))[0])[1]
    output_file_1 = "lsp_2d_benchmark.json" 
    result_folder = os.path.abspath("../benchmark_results/lsp/"+env_name)
    if not os.path.exists(result_folder):
      os.makedirs(result_folder)    
    json.dump(results, open(os.path.join(result_folder, output_file_1), 'w'), sort_keys=True)


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--database_folders', nargs='+', required=True)
  parser.add_argument('--num_envs', type=int, required=True)
  args = parser.parse_args()
  #Run the benchmark and save results
  run_benchmark(args.database_folders, args.num_envs)
  # print(results)