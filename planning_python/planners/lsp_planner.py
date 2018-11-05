#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time
from planning_python.data_structures import PlanningProblem
from planning_python.utils import helpers

class LSPPlanner(object):
  def __init__(self):
    self.initialized = False
  
  def initialize(self, problem, base_planner, policy=0, obs_cost = np.inf):
    """Initialize the planner with some planning problem"""
    assert problem.initialized == True, "Planning problem data structure has not been initialized"
    self.problem    = problem 
    self.env        = problem.env
    self.graph    = problem.graph
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
      self.env.initialize_plot(self.graph.node_to_state(self.start_node), self.graph.node_to_state(self.goal_node))#, grid_res = [self.graph.resolution[0], self.graph.resolution[1]])
    self.initialized = True
    self.iter = 0  #Number of planning iterations till solution found

  def plan(self, max_iters = np.inf, suppress_base_output=False):
    start_time = time.time()
    converged = False
    cand_path = None
    path_cost = np.inf
    num_evals = 0
    num_base_calls = 0
    run_base_planner = True

    while self.iter < max_iters:
      iter_start_time = time.time()
      base_plan_time = 0
      base_plan_expansions = 0
      if run_base_planner:
        if suppress_base_output:
          with helpers.nostdout():
            cand_path, path_cost, base_plan_expansions, base_plan_time, came_from, _, _ = self.base_planner.plan()
        else:
          cand_path, path_cost, base_plan_expansions, base_plan_time, came_from, _, _ = self.base_planner.plan()

        print('Iteration %d, base_planner_expansions %f, base planning time = %f, total time =  %f'%(self.iter, base_plan_expansions, base_plan_time, time.time() - iter_start_time))
        num_base_calls += 1

      e_selected = self.edge_selector(cand_path, self.policy)
      if len(e_selected) == 0:
        print('Path found')
        break
      curr_evals, run_base_planner = self.update_cost_and_e_eval(e_selected)
      num_evals += curr_evals
      self.reset_base_planner()


      if self.visualize and self.env is not None:
        self.env.plot_path(cand_path, 'solid', 'red', 1)
        [self.env.plot_edge(e, 'solid', 'green', 2) for e in e_selected]
      self.iter += 1

    plan_time = time.time() - start_time
    return cand_path, path_cost, num_evals, plan_time, self.iter, num_base_calls


  def edge_selector(self, path, policy=0):
    """Takes as input a path and a policy number and returns a list of edges to be evaluated"""
    e_selected = []
    if policy == 0:
      #SelectExpand policy
      e_selected = self.select_expand(path)
    elif policy == 1: 
      #SelectForward policy
      e_selected = self.select_forward(path)
    elif policy == 2:
      #SelectBackward policy
      e_selected = self.select_backward(path)
    elif policy == 3:
      #selectAlternate policy
      e_selected = self.select_alternate(path, self.iter)
    elif policy == 4:
      #Selectbisection
      e_selected = self.select_bisection(path, self.iter)
    return e_selected

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
        self.lazy_cost.add_edge(edge, cost)
        #we will set flag to run base planner again on next iteration only if any of the evaluated
        #edges has cost > w_est
        if cost > self.lazy_cost.w_est.get_cost(edge):
          run_next_iter = True
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
    self.lazy_cost.e_eval.clear()
    self.reset_base_planner()


  #Selection policies
  def select_expand(self, path):
    e_selected = []
    for edge in path:
      if edge not in self.lazy_cost.e_eval:
        #Look-up successor edges for first node in edge
        node = self.graph.state_to_node(edge[0])      
        if self.graph.edge_precalc_done:
          succs = self.graph.node_to_succs[node]
        else:  
          succs = self.graph.get_successors(node)
        for i,e in enumerate(succs):
          e_selected.append(e[1])
        break
    return e_selected

  def select_forward(self, path):
    e_selected = []
    for edge in path:
      if edge not in self.lazy_cost.e_eval:
        e_selected = [edge]
        break
    return e_selected

  def select_backward(self, path):
    e_selected = []
    for edge in reversed(path):
      if edge not in self.lazy_cost.e_eval:
        e_selected = [edge]
        break
    return e_selected 

  def select_alternate(self, path, iter):
    e_selected = []
    if iter%2 == 0:
      e_selected = self.select_backward(path)
    else:
      e_selected = self.select_forward(path)
    return e_selected

  def select_bisection(self, path):
    return [] 



