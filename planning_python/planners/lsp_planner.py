#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time

class LSPPlanner(object):
  def __init__(self):
    self.initialized = False
  
  def initialize(self, problem, base_planner):
    """Initialize the planner with some planning problem"""
    assert problem.initialized == True, "Planning problem data structure has not been initialized"
    self.curr_problem = problem 
    self.env        = problem.env
    self.lattice    = problem.lattice
    self.cost       = problem.cost
    self.lazy_cost  = problem.lazy_cost
    self.heuristic  = problem.heuristic
    self.visualize  = problem.visualize
    self.start_node = problem.start_n
    self.goal_node  = problem.goal_n
    self.heuristic_weight = problem.params['heuristic_weight']
    
    #Initialize the base planner for computing the shortest path
    base_planner_prob = problem
    base_planner_prob.cost = self.lazy_cost #Base planner should not use true cost
    base_planner_prob.env = None #Base planner should not evaluate edges
    self.base_planner = base_planner
    self.base_planner.initialize(base_planner_prob)

    if self.visualize:
      self.env.initialize_plot(self.lattice.node_to_state(self.start_node), self.lattice.node_to_state(self.goal_node))#, grid_res = [self.lattice.resolution[0], self.lattice.resolution[1]])
    self.initialized = True

  def plan(self):
    converged = False
    self.iter = 0
    self.e_eval = dict()
    while not converged:
      path, path_cost, curr_expansions, plan_time, self.came_from, self.cost_so_far, self.c_obs = self.base_planner.plan()
      self.iter += 1
      converged = True

    return path, path_cost, curr_expansions, plan_time, self.iter

  def collision_check(self, edges):
    return None

  def edge_selector(self, path, policy=0):
    if policy == 0:
      return path
    if policy == 1:
      return path[0]
    if policy == 2:
      return path[-1]
    else:
      return None

  # def 