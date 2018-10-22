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
    # self.e_eval = dict() #Edges that have been evaluated with their true cost

  def plan(self, max_iters = np.inf):
    start_time = time.time()
    converged = False
    cand_path = None
    path_cost = np.inf
    num_evals = 0
    run_base_planner = True
    while self.iter < max_iters:
      iter_start_time = time.time()
      base_plan_time = 0
      base_plan_expansions = 0
      if run_base_planner:
        cand_path, path_cost, base_plan_expansions, base_plan_time, _, _, _ = self.base_planner.plan()
      # if self.check_path_found(cand_path):
      #   print('Path found')
      #   break
      e_selected = self.edge_selector(cand_path, self.policy)
      if len(e_selected) == 0:
        print('Path found')
        break
      curr_evals, run_base_planner = self.update_cost_and_e_eval(e_selected)
      num_evals += curr_evals
      self.reset_base_planner()

      print('Iteration %d, base_planner_expansions %f, base planning time = %f, total time =  %f'%(self.iter, base_plan_expansions, base_plan_time, time.time() - iter_start_time))
      self.iter += 1

    plan_time = time.time() - start_time
    return cand_path, path_cost, num_evals, plan_time, self.iter


  def edge_selector(self, path, policy=0):
    """Takes as input a path and a policy number and returns a list of edges to be evaluated"""
    e_selected = []
    if policy == 0:
      #SelectExpand policy
      for edge in path:
        if edge not in self.lazy_cost.e_eval:
          #Look-up successor edges for first node in edge
          node = self.lattice.state_to_node(edge[0])      
          if self.lattice.edge_precalc_done:
            succs = self.lattice.node_to_succs[node]
          else:  
            succs = self.lattice.get_successors(node)
          for i,e in enumerate(succs):
            e_selected.append(e[1])
          break
    elif policy == 1: 
      #SelectForward policy
      for edge in path:
        if edge not in self.lazy_cost.e_eval:
          e_selected = [edge]
    elif policy == 2:
      #SelectBackward policy
      for edge in reversed(path):
        if edge not in self.lazy_cost.e_eval:
          e_selected = [edge]
    elif policy == 3:
      #SelectAlternate policy
      e_selected = [path[0]]
    return e_selected

  # def check_path_found(self, path):
  #   done = True
  #   for edge in path:
  #     if edge in self.e_eval:
  #       continue
  #     else:
  #       done = False
  #       break
  #   return done 

  def update_cost_and_e_eval(self, e_selected):
    num_eval = 0
    run_next_iter = False
    for edge in e_selected:
      if edge not in self.lazy_cost.e_eval:
        #Edge has not yet been evaluated
        isvalid, first_coll_state = self.env.is_edge_valid(edge)
        if isvalid:
          cost = self.true_cost.get_cost(edge)
        else:
          cost = self.obs_cost
        #we will set flag to run base planner again on next iteration only if any of the evaluated
        #edges has cost > w_est
        if cost > self.lazy_cost.w_est.get_cost(edge):
          run_next_iter = True
        self.lazy_cost.add_edge(edge, cost)
        num_eval += 1
      else:
        #Edge has already been evaluated once
        continue
    return num_eval, run_next_iter


  def reset_base_planner(self):
    self.base_planner.clear_planner()
    base_planner_prob = self.problem
    base_planner_prob.env  = None
    base_planner_prob.cost = self.lazy_cost
    self.base_planner.initialize(base_planner_prob)

  def reset(self):
    self.iter = 0
    self.e_eval.clear()