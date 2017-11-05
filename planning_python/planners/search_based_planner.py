#! /usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time

class SearchBasedPlanner(object):
  def __init__(self):
    self.initialized=False

  def initialize(self, problem):
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

    if self.visualize:
      self.env.initialize_plot(self.lattice.node_to_state(self.start_node), self.lattice.node_to_state(self.goal_node))#, grid_res = [self.lattice.resolution[0], self.lattice.resolution[1]])
    self.initialized = True

  def get_successors(self, node):
    """Given a node, query the lattice for successors, collision check the successors and return successor nodes, edges, costs, obstacle
    successors

    @param:   node          - tuple corresponsing to a discrete node 
    @return:  neighbors     - list of tuples where each tuple is a valid neighbor node of input
              costs         - costs of associated valid edges
              valid_edges   - list of collision free edges(continuous coords) coming out of the input node
              invalid_edges - a list of tuples where each tuple is of following form: (invalid edge, first invalid state along edge)
    """
    if self.lattice.edge_precalc_done:
      succs = self.lattice.node_to_succs[node]
    else:  
      succs = self.lattice.get_successors(node)
    neighbors = []     #Discrete nodes that are valid neighbors of current node
    costs = []         #Cost associated with each of the neighbors
    valid_edges =[]    #Valid edges coming out of the node
    invalid_edges = [] #Continuous states that are first states in collision along an edge
    for i,succ in enumerate(succs):
      succ_node = succ[0]
      succ_edge = succ[1]
      # print succ_edge
      isvalid, first_coll_state = self.env.is_edge_valid(succ_edge)
      if not isvalid:
        if first_coll_state:
          invalid_edges.append((succ_edge, first_coll_state))
        continue
      neighbors.append(succ_node)
      valid_edges.append(succ_edge)

      #If precalculated costs available, use them else calculate them
      if self.lattice.costs_precalc_done:
        costs.append(self.lattice.succ_costs[node][i])
      else:
        costs.append(self.cost.get_cost(succ_edge))
    #Visualize exansion if required
    if self.visualize:
      self.visualize_search(valid_edges, invalid_edges)
      
    return neighbors, costs, valid_edges, invalid_edges

  def get_predecessors(self, node):
    """Given a node, query the lattice for predecessors, collision check the predecessors and return predecessor nodes, edges, costs, obstacle
    predecessors

    @param:   node          - tuple corresponsing to a discrete node 
    @return:  neighbors     - list of tuples where each tuple is a valid neighbor node of input
              costs         - costs of associated valid edges
              valid_edges   - list of collision free edges(continuous coords) coming out of the input node
              invalid_edges - a list of tuples where each tuple is of following form: (invalid edge, first invalid state along edge)
    """
    if self.lattice.edge_precalc_done:
      preds = self.lattice.node_to_preds[node]
    else:  
      preds = self.lattice.get_predecessors(node)
    
    neighbors = []     #Discrete nodes that are valid neighbors of current node
    costs = []         #Cost associated with each of the neighbors
    valid_edges =[]    #Valid edges coming out of the node
    invalid_edges = [] #Continuous states that are first states in collision along an edge
    
    for i, pred in enumerate(preds):
      pred_node = pred[0]
      pred_edge = pred[1]
      isvalid, first_coll_state = self.env.is_edge_valid(pred_edge) #first_coll_state will be empty if the state simply lies outside workspace limits
      if not isvalid:
        if first_coll_state:
          invalid_edges.append((pred_edge, first_coll_state))
        continue
      neighbors.append(pred_node)
      valid_edges.append(pred_edge)
      if self.lattice.costs_precalc_done:
        costs.append(self.lattice.pred_costs[node][i])
      else:
        costs.append(self.cost.get_cost(pred_edge))
    if self.visualize:
      self.visualize_search(valid_edges, invalid_edges)
      
    return neighbors, costs, valid_edges, invalid_edges    


  def plan(self):
    """This is the main function which is called for solving a planning problem.
    This function is specific to a planner.
    """
    raise NotImplementedError
  
  def reconstruct_path(self, came_from, start_node, goal_node, cost_so_far):
    curr = goal_node
    path = []#[tuple(self.lattice.node_to_state(goal_node))]
    path_cost = cost_so_far[goal_node]
    
    while True:
      prev, edge = came_from[curr]
      if prev is None:
        break
      #Reverse the edge and append to the path
      # edge.reverse()
      # path += edge[1:] #Note that we need to ignore first element of reversed edge to avoid duplicate entries in the final path
      path.append(edge)
      curr = prev
    #Now reverse the entire path to get correct order  
    path.reverse()
    if self.visualize:
      self.env.plot_path(path, 'solid', 'red', 3)
      # plt.pause(1)
    return path, path_cost

  def get_heuristic(self, node_1, node_2):
    if self.heuristic == None:
      return 0
    s_1 = self.lattice.node_to_state(node_1)
    s_2 = self.lattice.node_to_state(node_2)
    h_val = self.heuristic.get_heuristic(s_1, s_2)
    return h_val
    
  def visualize_search(self, valid_edges, invalid_edges):
    self.env.plot_edges(valid_edges, 'solid', 'blue', 2)
    [self.env.plot_edge(e[0], 'solid', 'green', 2) for e in invalid_edges]
    return None
    
  
  def visualize_search_final(self, path, came_from, invalid_edges):
    return None

  def reset_problem(self, problem):
    """Reset the underlying problem that the planner solves while
    still persisting the information associated with the search tree"""
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

    if self.visualize:
      self.env.initialize_plot(self.lattice.node_to_state(self.start_node), self.lattice.node_to_state(self.goal_node))
    self.initialized = True    

  def clear_planner(self):
    """When this is called the planner clears information associated with the 
    previous tree search"""
    raise NotImplementedError

