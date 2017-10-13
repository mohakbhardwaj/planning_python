#!/usr/bin/env python
from collections import defaultdict
import numpy as np
from planning_python.data_structures.priority_queue import PriorityQueue
from planning_python.planners.planner import Planner


class Astar(Planner):
  def __init__(self):
    """Planner takes as input a planning problem object and returns
      the path and generated states
    @param problem   - planning problem object with following fields
    """
    super(Astar, self).__init__()

  def plan(self, max_expansions = 100000):
    assert self.initialized == True, "Planner has not been initialized properly. Please call initialize or reset_problem function before plan function"
    frontier = PriorityQueue()  #Initialize open list
    visited = defaultdict(lambda: np.inf)    #Initialized closed(visited) set
    c_obs = []                  #Initialize Cobs list
    
    came_from = {}              #Keep track of edges a node came from
    cost_so_far = defaultdict(lambda: np.inf)            #Keep track of cost of shortest path from start to each node visited so far
    came_from[self.start_node]= (None, None)
    cost_so_far[self.start_node] = 0.
    start_h_val = self.get_heuristic(self.start_node, self.goal_node, came_from, cost_so_far, list(visited), c_obs)
    frontier.put(self.start_node, 0 + self.heuristic_weight*start_h_val, self.heuristic_weight*start_h_val)

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
      #Check 3: Stop search if goal found
      if curr_node == self.goal_node:  
        print "Found goal"
        found_goal = True
        break
      
      #Step 3: If search has not ended, add neighbors of current node to frontier
      neighbors, edge_costs, valid_edges, invalid_edges = self.get_successors(curr_node)

      #Step 4: Update c_obs with collision checks performed
      c_obs.append(invalid_edges)
      g = cost_so_far[curr_node]
      
      for i, neighbor in enumerate(neighbors):
        new_g = g + edge_costs[i]
        if neighbor not in visited:
          if new_g < cost_so_far[neighbor]:
            came_from[neighbor] = (curr_node, valid_edges[i])
            cost_so_far[neighbor] = new_g
            h_val = self.heuristic_weight*self.get_heuristic(neighbor, self.goal_node, came_from, cost_so_far, list(visited), c_obs)
            f_val = new_g + h_val
            frontier.put(neighbor, f_val, h_val)
      
      #Step 5:increment number of expansions
      curr_expansions += 1

    if found_goal:
      path, path_cost = self.reconstruct_path(came_from, self.start_node, self.goal_node, cost_so_far)
    else:
      print ('Found no solution, priority queue empty')
    return path, path_cost, curr_expansions, came_from, cost_so_far, c_obs
    
  
  def clear_planner(self):
    return None