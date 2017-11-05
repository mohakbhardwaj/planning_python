#!/usr/bin/env python
"""Runs a backward Djikstra to all nodes on the graph. Calculates optimal cost to go for every node """
from collections import defaultdict
import numpy as np
import time
from planning_python.data_structures.priority_queue import PriorityQueue
from planning_python.planners.search_based_planner import SearchBasedPlanner


class ValueIteration(SearchBasedPlanner):
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

    super(ValueIteration, self).__init__()

  def plan(self, max_expansions = 100000):
    assert self.initialized == True, "Planner has not been initialized properly. Please call initialize or reset_problem function before plan function"
    start_t = time.time()

    self.came_from[self.goal_node]= (None, None)
    self.cost_so_far[self.goal_node] = 0.
    
    self.frontier.put(self.goal_node, 0.0)

    curr_expansions = 0         #Number of expansions done
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
      f, _, curr_node = self.frontier.get()
      
      if curr_node in self.visited:
        continue
      #Step 2: Add to visited
      self.visited[curr_node] = 1
      
      
      #Step 3: If search has not ended, add neighbors of current node to frontier. We call get_predecessors instead of get_successors
      neighbors, edge_costs, valid_edges, invalid_edges = self.get_predecessors(curr_node)

      g = self.cost_so_far[curr_node]
      
      for i, neighbor in enumerate(neighbors):
        new_g = g + edge_costs[i]
        if neighbor not in self.visited:
          if neighbor not in self.cost_so_far or new_g < self.cost_so_far[neighbor]:
            self.came_from[neighbor] = (curr_node, valid_edges[i])
            self.cost_so_far[neighbor] = new_g
            f_val = new_g
            self.frontier.put(neighbor, f_val)
      
      #Step 5:increment number of expansions
      curr_expansions += 1
    plan_time = time.time() - start_t

    return path, path_cost, curr_expansions, plan_time, self.came_from, self.cost_so_far, self.c_obs
    
  
  def clear_planner(self):
    self.frontier.clear()
    self.visited = {}
    self.c_obs = []
    self.cost_so_far = {}
    self.came_from = {}