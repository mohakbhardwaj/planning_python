#!/usr/bin/env python
import os
import sys
import numpy as np
from threading import Thread
from sets import Set
import time
sys.path.insert(0, os.path.abspath('..'))
from planning_python.data_structures import PriorityQueue
from Planner import *
from planning_python.utils import heuristic_functions, planner_utils#, visualization_utils

# from graphs import *
class MHAstar(Planner):
  def __init__(self, problem, visualize = False, do_greedy = False):
    """Planner takes as input a planning prblem object and a base heuristic function object and returns
      the path and generated states"""
    Planner.__init__(self, problem, visualize)
    self.heuristic_functions = [heuristic_functions.Euclidean,heuristic_functions.Manhattan,heuristic_functions.ObsDistance]
    self.do_greedy = do_greedy

  def plan(self, max_expansions = 100000):
    start_time = time.time()
    start = self.problem.start
    goal = self.problem.goal
    start_id = self.problem.g.configuration_to_node_id(start)
    goal_id = self.problem.g.configuration_to_node_id(goal)
    frontier1 = PriorityQueue()
    frontier2 = PriorityQueue()
    frontier3 = PriorityQueue()
    curr_expansions = 0
    num_rexpansions =0
    came_from = {}
    # visited = Set()
    cost_so_far = {}
    found_goal = False
    
    path = []
    motions = []
    path_cost = np.inf
    c_obs = []
  
    frontier1.put(start_id, None, None, self.heuristic_functions[0](start,goal),self.heuristic_functions[0](start,goal), 0., 0.)
    frontier2.put(start_id, None, None, self.heuristic_functions[1](start,goal),self.heuristic_functions[1](start,goal), 0., 0.)
    frontier3.put(start_id, None, None, self.heuristic_functions[2](start,c_obs),self.heuristic_functions[2](start,c_obs), 0., 0.)    
    cost_so_far[start_id] = 0.
    
    if self.visualize:
      [self.problem.plot_state(x) for x in [start_id, goal_id]]
      self.visualizer.render(self.problem.plot)

    frontiers=[frontier1, frontier2, frontier3] 
    while (not frontier1.empty()) or (not frontier2.empty()) or (not frontier3.empty()) :
      #Check 1: Stop search if frontier gts too large
      if curr_expansions >= max_expansions:
        print "Max Expansions Done"
        # self.planning_done = True
        break
      #Check 2: Stop search if open list gets too large
      if frontier1.size() > 500000 or frontier2.size() > 500000 or frontier3.size() > 50000:
        print "Timeout."
        break
      idx = curr_expansions%len(frontiers)

      total_cost, h_val, cost, _, current_id, previous_id, move = frontiers[idx].get()
      
      if curr_expansions%100 == 0: print("%i/%i"%(curr_expansions, max_expansions))

      if current_id in came_from:
        num_rexpansions +=1
        continue
      
      came_from[current_id] = (previous_id, move)
      cost_so_far[current_id] = cost
      # visited.add(current_id)

      #Check 3: Stop search if goal found
      if self.problem.g.states_close(current_id, goal_id):
        print "Found goal"
        found_goal = True
        break
      
      # if self.planning_done: break
      
      neighbor_ids, movements, obs_neighbors = self.problem.g.get_successors(current_id)
      [c_obs.append(self.problem.g.node_id_to_configuration(obs_id)) for obs_id in obs_neighbors]
      curr_expansions += 1
      for (next_id, i) in zip(neighbor_ids, movements):
        if next_id not in came_from:
          new_cost = cost + self.problem.g.cost(current_id, next_id, i)
          for i in xrange(len(frontiers)):
            if i == 2: heuristic_val = self.heuristic_functions[i](self.problem.g.node_id_to_configuration(next_id), c_obs)
            else: heuristic_val = self.heuristic_functions[i](self.problem.g.node_id_to_configuration(next_id), goal) 
            total_cost = heuristic_val
          #total_cost = g+h is not running greedy best first search
            if not self.do_greedy:
              total_cost += new_cost 
            frontiers[i].put(next_id, current_id, i, total_cost, heuristic_val, new_cost, 0)
          #Solely for visualization purposes
          if self.visualize:
            self.problem.plot_edge(current_id, next_id, i)
            self.visualizer.render(self.problem.plot)
    
    if found_goal:
      path, motions, path_cost = self.problem.g.reconstruct_path(came_from, start_id, current_id, cost_so_far)
      if self.visualize:
        self.problem.plot_path(path, motions)
        self.visualizer.render(self.problem.plot)
        time.sleep(1)
        self.visualizer.close()
    
    print "Time taken: ", time.time() - start_time
    print "Number of Expansions: ", curr_expansions
    print "Number of Rexpansions: ", num_rexpansions
    return path, motions, path_cost, cost_so_far, came_from, curr_expansions
  
