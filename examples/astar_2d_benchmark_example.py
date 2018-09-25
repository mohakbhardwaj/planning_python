#!/usr/bin/env python
"""This file takes as input an environment database folder and num_envs, runs a-star on the database with different heuristic 
weights and returns the results. 

Author: Mohak Bhardwaj
Date: October 6, 2017
"""
import argparse
from collections import defaultdict
import json
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
sys.path.insert(0, "..")
import time
from planning_python.environment_interface.env_2d import Env2D
from planning_python.state_lattices.common_lattice.xy_analytic_lattice import XYAnalyticLattice
from planning_python.cost_functions.cost_function import PathLengthNoAng
from planning_python.heuristic_functions.heuristic_function import EuclideanHeuristicNoAng, ManhattanHeuristicNoAng
from planning_python.data_structures.planning_problem import PlanningProblem
from planning_python.planners.astar import Astar

x_lims = [0, 200]
y_lims = [0, 200]

env_params = {'x_lims': x_lims, 'y_lims': y_lims}
lattice_params = {'x_lims': x_lims, 'y_lims': y_lims, 'resolution': [1, 1], 'origin': (0, 0), 'rotation': 0, 'connectivity': 'eight_connected', 'path_resolution': 1}
h_weight_list = [1] #We will run Astar by putting differen weights on the heuristic each time
cost_fn = PathLengthNoAng()
heuristic_fn = EuclideanHeuristicNoAng()
lattice = XYAnalyticLattice(lattice_params)
planner = Astar()
start_n = lattice.state_to_node((0,0))
goal_n = lattice.state_to_node((199, 199))
visualize = False

def run_benchmark(database_folders=[], num_envs=1):
  global env_params, lattice_params, cost_fn, heuristic_fn, lattice, planner, start_n, goal_n, h_weight_list, visualize
     
  lattice.precalc_costs(cost_fn)
  e = Env2D()
  print('Running benchmark')

  for folder in database_folders:
    results= defaultdict(list)
    
    for i in range(num_envs):
      curr_env_file = os.path.join(os.path.abspath(folder), str(i)+'.png')
      e.initialize(curr_env_file, env_params)
    
      for h_weight in h_weight_list: 
        prob_params = {'heuristic_weight': h_weight}
        prob = PlanningProblem(prob_params)
        prob.initialize(e, lattice, cost_fn, heuristic_fn, start_n, goal_n, visualize=visualize)
        planner.initialize(prob) 
        path, path_cost, num_expansions, plan_time, came_from, cost_so_far, c_obs = planner.plan()
        results[h_weight].append((num_expansions,plan_time))
        planner.clear_planner() #clear the planner in the end
    
    env_name = os.path.split(os.path.split(os.path.abspath(folder))[0])[1]
    output_file_1 = "astar_2d_benchmark.json" 
    json.dump(results, open(os.path.join(os.path.abspath("../benchmark_results/astar/"+env_name), output_file_1), 'w'), sort_keys=True)
    output_file_2 = "astar_2d_enchmark_averaged.json"
    
    #Calculate average expansions
    avg_results = defaultdict(list)
    for k,v in results.iteritems():
      avg_expansions = 0
      avg_time = 0
      for exp, t in v:
        avg_expansions += exp
        avg_time += t
      avg_expansions /= num_envs
      avg_time /= num_envs
      avg_results[k] = (avg_expansions, avg_time)
    json.dump(avg_results, open(os.path.join(os.path.abspath("../benchmark_results/astar/"+env_name), output_file_2), 'w'), sort_keys=True)   


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--database_folders', nargs='+', required=True)
  parser.add_argument('--num_envs', type=int, required=True)
  args = parser.parse_args()
  #Run the benchmark and save results
  run_benchmark(args.database_folders, args.num_envs)
  # print(results)
