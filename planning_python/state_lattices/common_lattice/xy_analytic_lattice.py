#!/usr/bin/env python
import numpy as np
from math import floor, cos, sin
from sets import Set
from planning_python.state_lattices.state_lattice import StateLattice

class XYAnalyticLattice(StateLattice):
  def __init__(self, params):
    #unpack all parameters
    self.ndims           = 2
    self.x_lims          = params['x_lims']
    self.y_lims          = params['y_lims']
    self.connectivity    = params['connectivity']
    self.resolution      = np.asarray(params['resolution'])
    self.origin          = np.asarray(params['origin']) #origin of lattice in world
    self.rotation        = params['rotation'] #rotation of lattice about z-axis w.r.t world
    self.path_resolution = params['path_resolution']
    

    
    #Define grid connectivity. This describes what nodes in the discrete
    #graph are connected to what other nodes.
    if self.connectivity == "four_connected":
      self.max_movement_id = 4
      self.children = np.array([(1,0), (0, -1), (-1, 0), (0, 1)])
      self.predecessors = self.children
    if self.connectivity == "eight_connected":
      self.max_movement_id = 8
      self.children = np.array([(1, 1), (1,-1), (-1, -1), (-1, 1), (1, 0), (0, -1),  (-1, 0), (0, 1)])
      self.predecessors = self.children
    
    StateLattice.__init__(self, self.ndims, [self.x_lims[0], self.y_lims[0]], [self.x_lims[1], self.y_lims[1]], self.resolution)

    #Precalculate successors and predecessors
    self.node_to_succs = dict()
    self.node_to_preds = dict()
    self.succ_costs = dict()
    self.pred_costs = dict()
    self.edge_precalc_done = False
    self.costs_precalc_done = False
    

  def node_to_state(self, node):
    """Convert a discrete node to a world state taking origin and rotation of lattice into account

    @param  node  - a tuple containing discrete (x,y) coordinates 
    @return state - np array of state in continuous world coordinates 

    """
    xd = node[0]
    yd = node[1]
    state = self.origin + np.array([self.resolution[0]*(cos(self.rotation)*xd - sin(self.rotation)*yd),
                                    self.resolution[1]*(sin(self.rotation)*xd + cos(self.rotation)*yd)])
    return state
  
  def state_to_node(self, state):
    """Convert a continuous state(in world coordinates) to a discrete node()in lattice coordinates)"""
    node = np.asarray(state)
    node -= self.origin
    node = np.divide(node, self.resolution)
    #we need to rotate the state to lattice coordinates
    node = np.array([ cos(self.rotation)*node[0] + sin(self.rotation)*node[1], 
                     -sin(self.rotation)*node[0] + cos(self.rotation)*node[1]]) 
    node = node.astype(int, copy=False) 
    return tuple(node)

  def get_edge(self, parent_node, child_node):
    """Given two discrete nodes, returns an edge(sequence of waypoints) connecting them. 
    The returned edge includes the start and end points
     
    @param : parent_node - discrete node which is the parent
    @param : child_node  - discrete node which is the child
    @return: edge        - list of waypoints(continuous) in world coordinates
    """
    parent_state = self.node_to_state(parent_node)
    child_state = self.node_to_state(child_node)
    num_points = self.distance_bw_states(parent_state, child_state)/self.path_resolution
    return self.interpolate(parent_state, child_state, num_points)
    

  def get_successors(self, node):
    """Given a discrete node in the lattice, returns the child nodes (discrete) and corresponding
    edges (continuous states)

    The edges use the parameter path_resolution to interpolate between two nodes. 
    
    @param node    - node for which successors are to be queried
    @return succs  - a list of lists, where the first element is a child node (discrete) and second element is 
                     an edge connecting current node to child node (continuous states) 
    """
    succs = []
    parent_state = self.node_to_state(node)
    for it in self.children:
      child_node  = (node[0] + it[0], node[1] + it[1])
      child_state = self.node_to_state(child_node)
      edge = self.interpolate(parent_state, child_state, self.distance_bw_states(parent_state, child_state)/self.path_resolution)
      succs.append([child_node, edge])
    return succs

  def get_predecessors(self, node):
    """Given a discrete node in the lattice, returns all the predecessor nodes (discrete) and corresponding
    edges (continuous states)

    The edges use the parameter path_resolution to interpolate between two nodes. 
    
    @param node    - node for which successors are to be queried
    @return preds  - a list of lists, where the first element is a parent node (discrete) and second element is 
                     an edge connecting current node to parent node (continuous states) 
    """    
    preds = []
    child_state = self.node_to_state(node)
    for it in self.predecessors:
      parent_node = (node[0] + it[0], node[1] + it[1])
      parent_state = self.node_to_state(parent_node)
      edge = self.interpolate(child_state, parent_state, self.distance_bw_states(child_state, parent_state)/self.path_resolution)
      preds.append([parent_node, edge])
    return preds


  def interpolate(self, s1, s2, num_points):
    """Returns a sequence of states between two states 
    given a motion that connects the two states.

    We use a shooting method type approach to interpolate between the two states

    @param: s1         - parent state
    @param: s2         - child state
    @params motion     - control input connecting the two states
    @params num_points - number of intermediate points (excluding the end points)
    @return edge       - sequence of states (including end points) 
    """
    edge = [tuple(s1)]
    length = self.distance_bw_states(s1, s2)
    d_l = length/num_points
    curr_len = d_l
    while True:
      #Do linear interpolation
      temp = [s1[i] + (s2[i] - s1[i])*(curr_len/length) for i in range(self.ndims)]
      #Update curr_len
      curr_len+= d_l
      if curr_len > length: break
      #Append temp to edge
      edge.append(tuple(temp))
    #Add final state to edge
    edge.append(tuple(s2))
    return edge
  
  def distance_bw_states(self, s1, s2):
    return np.linalg.norm(s1-s2)

  def enumerate_lattice(self):
    """For every discrete node in the lattice, we store a dictionary of [successor nodes, successor edges] and 
    [predecessor_node, predecessor_edges]"""
    node_to_succs = dict()
    node_to_preds = dict()
    start_node = self.state_to_node([self.x_lims[0], self.y_lims[0]])
    for i in range(self.num_cells[0]):
      x = start_node[0] + i
      for j in range(self.num_cells[1]):
        y = start_node[1] + j
        node = (x,y)
        node_to_succs[node] = self.get_successors(node)
        node_to_preds[node] = self.get_predecessors(node)
    
    assert(len(node_to_succs.keys()) == self.total_cells), "Did not enumerate all possible successor cells"
    assert(len(node_to_preds.keys()) == self.total_cells), "Did not enumerate all possible predecessor cells"
    return node_to_succs, node_to_preds

  
  def precalc_edges(self):
    self.node_to_succs, self.node_to_preds = self.enumerate_lattice()
    self.edge_precalc_done = True

  def precalc_costs(self, cost_fn):
    #Do precalculation of costs here
    print('Precalculating Edges and Costs')
    if not self.edge_precalc_done:
      self.precalc_edges()
    for node in self.node_to_succs:
      succs = self.node_to_succs[node]
      costs = []
      for succ_node, succ_edge in succs:
        costs.append(cost_fn.get_cost(succ_edge))
      self.succ_costs[node] = costs
      
      preds = self.node_to_preds[node]
      costs = []
      for pred_node, pred_edge in preds:
        costs.append(cost_fn.get_cost(pred_edge))
      self.pred_costs[node] = costs

    self.costs_precalc_done = True  