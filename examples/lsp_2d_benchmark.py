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

import sys
sys.path.insert(0, "..")
import matplotlib.pyplot as plt
import numpy as np
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.state_lattices.common_lattice.xy_analytic_lattice import XYAnalyticLattice
from planning_python.cost_functions.cost_function import PathLengthNoAng, UnitCost, LazyCost
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners import LSPPlanner, Astar
from planning_python.utils import helpers
import os


NUM_POLICIES = 5 #We are going to evaluate all the naive policies defined in the paper
x_lims = [0, 200] # low(inclusive), upper(exclusive) extents of world in x-axis
y_lims = [0, 200] # low(inclusive), upper(exclusive) extents of world in y-axis

env_params = {'x_lims': x_lims, 'y_lims': y_lims}
lattice_params = {'x_lims': x_lims, 'y_lims': y_lims, 'resolution': [1, 1], 'origin': (0, 0), 'rotation': 0, 'connectivity': 'eight_connected', 'path_resolution': 1}
prob_params = {'heuristic_weight': 1.0}

lattice = XYAnalyticLattice(lattice_params)
true_cost_fn = PathLengthNoAng()  #Penalize length of path
heuristic_fn = EuclideanHeuristicNoAng()      
lazy_cost_fn = LazyCost(PathLengthNoAng(), weight=1)
planner = LSPPlanner()
base_planner = Astar()
start_n = lattice.state_to_node((0,0))
goal_n = lattice.state_to_node((199, 199))
visualize = False


def run_benchmark(database_folders=[], num_envs=1):
  global env_params, lattice_params, prob_params, true_cost_fn, heuristic_fn, lazy_cost_fn, lattice, planner, base_planner, start_n, goal_n, NUM_POLICIES, visualize
  # lattice.precalc_costs(cost_fn)
  from collections import defaultdict
  e = Env2D()
  print('Running benchmark')

  for folder in database_folders:
    results= defaultdict(list)
    
    for i in range(num_envs):
      curr_env_file = os.path.join(os.path.abspath(folder), str(i)+'.png')
      print('Curr env number %d'%i)
      e.initialize(curr_env_file, env_params)
    
      for policy in range(NUM_POLICIES):
        print('Curr policy number %d', policy) 
        prob = PlanningProblem(prob_params)
        prob.initialize(e, lattice, true_cost_fn, heuristic_fn, start_n, goal_n, visualize=visualize)
        prob.set_lazy_cost(lazy_cost_fn)
        planner.initialize(prob, base_planner, policy=1) 
        path, path_cost, num_edge_evals, plan_time, num_iters, num_base_calls = planner.plan(max_iters=np.inf, suppress_base_output=True)
        results[policy].append((path_cost, num_edge_evals, num_iters, num_base_calls, plan_time))
        print('Path: ', path)
        print('Path cost: ', path_cost)
        print('Num edge evaluations: ', num_edge_evals)
        print('Time taken: ', plan_time)
        print('Num planning iterations', num_iters)
        print('Num base planner calls', num_base_calls)
        planner.reset() #clear the planner in the end
    
    env_name = os.path.split(os.path.split(os.path.abspath(folder))[0])[1]
    output_file_1 = "lsp_2d_benchmark.json" 
    result_folder = os.path.abspath("../benchmark_results/lsp/"+env_name)
    if not os.path.exists(result_folder):
      os.makedirs(results_folder)
    json.dump(results, open(os.path.join(os.path.abspath("../benchmark_results/lsp/"+env_name), output_file_1), 'w'), sort_keys=True)
    # output_file_2 = "lsp_2d_benchmark_averaged.json"
    
    # #Calculate average expansions
    # avg_results = defaultdict(list)
    # for k,v in results.iteritems():
    #   avg_expansions = 0
    #   avg_time = 0
    #   for exp, t in v:
    #     avg_expansions += exp
    #     avg_time += t
    #   avg_expansions /= num_envs
    #   avg_time /= num_envs
    #   avg_results[k] = (avg_expansions, avg_time)
    # json.dump(avg_results, open(os.path.join(os.path.abspath("../benchmark_results/lsp/"+env_name), output_file_2), 'w'), sort_keys=True)   



if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--database_folders', nargs='+', required=True)
  parser.add_argument('--num_envs', type=int, required=True)
  args = parser.parse_args()
  #Run the benchmark and save results
  run_benchmark(args.database_folders, args.num_envs)
  # print(results)