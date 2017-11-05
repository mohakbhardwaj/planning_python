#!/usr/bin/env python
"""Struct to define a planning problem which will be solved by a planner"""


class PlanningProblem:
  def __init__(self,  params):
    self.initialized = False
    self.params = params

  def initialize(self, env=None, lattice=None, cost=None, heuristic=None, start_n=None, goal_n=None, visualize=False):
    self.env         = env
    self.lattice     = lattice
    self.cost        = cost
    self.heuristic   = heuristic
    self.start_n     = start_n
    self.goal_n      = goal_n
    self.visualize   = visualize
    self.initialized = True
    print('Planning Problem Initialized')

  def reset_env(self, env):
    """Given the same lattice, cost, heuristic and params, reset the underlying environment"""
    self.env = env
  
  def reset_heuristic(self, heuristic):
    """Reset the heuristic function being used"""
    self.heuristic = heuristic

  
  