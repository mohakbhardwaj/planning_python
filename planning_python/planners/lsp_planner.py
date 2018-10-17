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
    self.heuristic  = problem.heuristic
    self.visualize  = problem.visualize
    self.start_node = problem.start_n
    self.goal_node  = problem.goal_n
    self.heuristic_weight = problem.params['heuristic_weight']
    
    #Initialize the base planner for computing the shortest path
    base_planner_prob = problem
    base_planner_prob.env = None #We don't want the base planner to do any collision checks
    self.base_planner = base_planner

    if self.visualize:
      self.env.initialize_plot(self.lattice.node_to_state(self.start_node), self.lattice.node_to_state(self.goal_node))#, grid_res = [self.lattice.resolution[0], self.lattice.resolution[1]])
    self.initialized = True  