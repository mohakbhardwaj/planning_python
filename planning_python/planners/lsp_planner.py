#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time
from planning_python.data_structures import PlanningProblem

class LSPPlanner(object):
  def __init__(self):
    self.initialized = False
  
  def initialize(self, problem, base_planner, policy=0, obs_cost = np.inf):
    """Initialize the planner with some planning problem"""
    assert problem.initialized == True, "Planning problem data structure has not been initialized"
    self.problem    = problem 
    self.env        = problem.env
    self.lattice    = problem.lattice
    self.true_cost  = problem.cost
    self.lazy_cost  = problem.lazy_cost
    self.heuristic  = problem.heuristic
    self.visualize  = problem.visualize
    self.start_node = problem.start_n
    self.goal_node  = problem.goal_n
    self.policy = policy
    self.heuristic_weight = problem.params['heuristic_weight']
    
    #Initialize the base planner for computing the shortest path
    base_planner_prob = problem
    base_planner_prob.cost = self.lazy_cost #Base planner should not use true cost
    base_planner_prob.env = None #Base planner should not evaluate edges
    self.base_planner = base_planner
    self.base_planner.initialize(base_planner_prob)
    self.obs_cost = obs_cost #Cost to be given to edges in collision

    if self.visualize:
      self.env.initialize_plot(self.lattice.node_to_state(self.start_node), self.lattice.node_to_state(self.goal_node))#, grid_res = [self.lattice.resolution[0], self.lattice.resolution[1]])
    self.initialized = True
    self.iter = 0  #Number of planning iterations till solution found
    self.e_eval = dict() #Edges that have been evaluated with their true cost

  def plan(self, max_iters = np.inf):
    start_time = time.time()
    converged = False
    cand_path = None
    path_cost = np.inf
    num_evals = 0
    while self.iter < max_iters:
      cand_path, path_cost, _, _, _, _, _ = self.base_planner.plan()
      if self.check_path_found(cand_path):
        print('Path found')
        break
      e_selected = self.edge_selector(cand_path, self.policy)
      num_evals += self.update_cost_and_e_eval(e_selected)
      self.reset_base_planner()
      self.iter += 1

    plan_time = time.time() - start_time
    return cand_path, path_cost, num_evals, plan_time, self.iter


  def edge_selector(self, path, policy=0):
    """Takes as input a path and a policy number and returns a list of edges to be evaluated"""
    if policy == 0:
      return path
    if policy == 1:
      return [path[0]]
    if policy == 2:
      return [path[-1]]
    else:
      return None

  def check_path_found(self, path):
    done = True
    for edge in path:
      if edge in self.e_eval:
        continue
      else:
        done = False
        break
    return done 

  def update_cost_and_e_eval(self, e_selected):
    num_eval = 0
    for edge in e_selected:
      if edge not in self.e_eval:
        #Edge has not yet been evaluated
        isvalid, first_coll_state = self.env.is_edge_valid(edge)
        if isvalid:
          cost = self.true_cost.get_cost(edge)
        else:
          cost = self.obs_cost
        self.e_eval[edge] = cost
        self.lazy_cost.add_edge(edge, cost)
        num_eval += 1
      else:
        #Edge has already been evaluated once
        continue
    return num_eval



  def reset_base_planner(self):
    self.base_planner.clear_planner()
    base_planner_prob = self.problem
    base_planner_prob.env  = None
    base_planner_prob.cost = self.lazy_cost
    self.base_planner.initialize(base_planner_prob)

  def reset(self):
    self.iter = 0
    self.e_eval.clear()