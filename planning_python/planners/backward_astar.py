#!/usr/bin/env python
from collections import defaultdict
import numpy as np
import time
from planning_python.data_structures.priority_queue import PriorityQueue
from planning_python.planners.search_based_planner import SearchBasedPlanner


class BackwardAstar(SearchBasedPlanner):
  def __init__(self):
    """Planner takes as input a planning problem object and returns
      the path and generated states
    @param problem   - planning problem object with following fields
    """
    super(BackwardAstar, self).__init__()

  def plan(self, max_expansions = 100000):
    assert self.initialized == True, "Planner has not been initialized properly. Please call initialize or reset_problem function before plan function"
    start_t = time.time()
    frontier = PriorityQueue()  #Initialize open list
    visited = dict()   #Initialized closed(visited) set
    c_obs = []                  #Initialize Cobs list
    
    came_from = {}              #Keep track of edges a node came from
    cost_so_far = dict()           #Keep track of cost of shortest path from start to each node visited so far
    
    came_from[self.goal_node]= (None, None)
    cost_so_far[self.goal_node] = 0.
    
    goal_h_val = self.get_heuristic(self.goal_node, self.start_node, came_from, cost_so_far, list(visited), c_obs)
    frontier.put(self.goal_node, 0 + self.heuristic_weight*goal_h_val, self.heuristic_weight*goal_h_val)

    curr_expansions = 0         #Number of expansions done
    num_rexpansions =0

    found_goal = False
        
    path =[]
    path_cost = np.inf
    while not frontier.empty():
      #Check 1: Stop search if frontier gets too large
      if curr_expansions >= max_expansions:
        print("Max Expansions Done.")
        break
      #Check 2: Stop search if open list gets too large
      if frontier.size() > 500000:
        print("Timeout.")
        break
      
      #Step 1: Pop the best node from the frontier
      f, h, curr_node = frontier.get()
      
      if curr_node in visited:
        continue
      #Step 2: Add to visited
      visited[curr_node] = 1
      
      #Check 3: Stop search if start found
      if curr_node == self.start_node:
        print "Found goal"
        found_goal = True
        break
      
      #Step 3: If search has not ended, add neighbors of current node to frontier. We call get_predecessors instead of get_successors
      neighbors, edge_costs, valid_edges, invalid_edges = self.get_predecessors(curr_node)

      #Step 4: Update c_obs with collision checks performed
      c_obs.append(invalid_edges)
      g = cost_so_far[curr_node]
      
      for i, neighbor in enumerate(neighbors):
        new_g = g + edge_costs[i]
        if neighbor not in visited:
          if neighbor not in cost_so_far or new_g < cost_so_far[neighbor]:
            came_from[neighbor] = (curr_node, valid_edges[i])
            cost_so_far[neighbor] = new_g
            h_val = self.heuristic_weight*self.get_heuristic(neighbor, self.goal_node)
            f_val = new_g + h_val
            frontier.put(neighbor, f_val, h_val)
      
      #Step 5:increment number of expansions
      curr_expansions += 1
 
    if found_goal:
      path, path_cost = self.reconstruct_path(came_from, self.goal_node, self.start_node, cost_so_far)
    else:
      print ('Found no solution, priority queue empty')
    plan_time = time.time() - start_t
    return path, path_cost, curr_expansions, plan_time, came_from, cost_so_far, c_obs
    
  
  def clear_planner(self):
    return None