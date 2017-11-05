#!/usr/bin/env python
from collections import defaultdict
import numpy as np
import time
from planning_python.data_structures.priority_queue import PriorityQueue
from planning_python.planners.search_based_planner import SearchBasedPlanner


class GreedyPlanner(SearchBasedPlanner):
  def __init__(self):
    """Planner takes as input a planning problem object and returns
      the path and generated states
    @param problem   - planning problem object with following fields
    """
    self.frontier = PriorityQueue() #Open list
    self.visited = {} #Keep track of visited cells
    self.c_obs = []  #Keep track of collision checks done so far
    self.cost_so_far = {} #Keep track of cost of path to the node
    self.came_from = {} #Keep track of parent during search

    super(GreedyPlanner, self).__init__()

  def plan(self, max_expansions = 100000):
    assert self.initialized == True, "Planner has not been initialized properly. Please call initialize or reset_problem function before plan function"
    start_t = time.time()
    self.came_from[self.start_node]= (None, None)
    self.cost_so_far[self.start_node] = 0.
    start_h_val = self.get_heuristic(self.start_node, self.goal_node)
    self.frontier.put(self.start_node, start_h_val)

    curr_expansions = 0         #Number of expansions done
    num_rexpansions = 0
    found_goal = False
    path =[]
    path_cost = np.inf

    while not self.frontier.empty():
      #Check 1: Stop search if frontier gets too large
      if curr_expansions >= max_expansions:
        print("Max Expansions Done.")
        break
      #Check 2: Stop search if open list gets too large
      if self.frontier.size() > 500000:
        print("Timeout.")
        break
      
      #Step 1: Pop the best node from the frontier
      h, _, curr_node = self.frontier.get()
      if curr_node in self.visited:
        continue
      #Step 2: Add to visited and increment expansions
      self.visited[curr_node] = 1
      curr_expansions += 1
      
      #Step 3: If search has not ended, add neighbors of current node to frontier
      neighbors, edge_costs, valid_edges, invalid_edges = self.get_successors(curr_node)

      #Step 4: Update c_obs with collision checks performed
      # self.c_obs.append(invalid_edges)
      g = self.cost_so_far[curr_node]
      for i, neighbor in enumerate(neighbors):
        new_g = g + edge_costs[i]
        if neighbor not in self.visited:
          if neighbor not in self.cost_so_far or new_g <= self.cost_so_far[neighbor]:
            self.came_from[neighbor] = (curr_node, valid_edges[i])
            self.cost_so_far[neighbor] = new_g
            
            if neighbor == self.goal_node: 
              print "Found goal"
              found_goal = True
              break
            
            h_val = self.get_heuristic(neighbor, self.goal_node)
            self.frontier.put(neighbor, h_val)
      
      if found_goal: break


    if found_goal:
      path, path_cost = self.reconstruct_path(self.came_from, self.start_node, self.goal_node, self.cost_so_far)
    else:
      print ('Found no solution, priority queue empty')
    plan_time = time.time() - start_t

    return path, path_cost, curr_expansions, plan_time, self.came_from, self.cost_so_far, self.c_obs
    
  
  def clear_planner(self):
    self.frontier.clear()
    self.visited = {}
    self.c_obs = []
    self.cost_so_far = {}
    self.came_from = {}

    