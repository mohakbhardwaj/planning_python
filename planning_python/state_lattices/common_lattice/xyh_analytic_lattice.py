#!/usr/bin/env python
import dubins
import numpy as np
from math import sqrt, sin, cos, pi, floor, radians, copysign
from sets import Set
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

from planning_python.state_lattices.state_lattice import StateLattice
from planning_python.utils import angles

class XYHAnalyticLattice(StateLattice):
  def __init__(self, params):
    self.ndims           = 3
    self.x_lims          = params['x_lims']                #bounds of lattice in x axis
    self.y_lims          = params['y_lims']                #bounds of lattice in y axis
    self.connectivity    = params['connectivity']          #Connectivity of the lattice i.e dubins_turn_90 or dubins_turn_45 etc.
    self.origin          = np.asarray(params['origin'])    #Origin of the lattice wrt world
    self.rotation        = params['rotation']              #Rotation of the lattice wrt world
    self.radius          = params['radius']                #Fixed turning radius of the vehicle
    self.path_resolution = params['path_resolution']       #Resolution to do collision checking at

    if self.connectivity == "dubins_turn_90":
      self.num_heading = 4
      self.resolution = np.array([params['radius'], params['radius'], np.pi/2.0]) #Resolution of lattice in different directions
      self.children = [[] for i in range(self.num_heading)]
      #heading 0
      self.children[0].append((1, 0, 0)) #straight
      self.children[0].append((1, 1, 1)) #left turn
      self.children[0].append((1, -1, 3)) #right turn
      #heading 1
      self.children[1].append((0, 1, 1)) #straight
      self.children[1].append((-1, 1, 2)) #left turn
      self.children[1].append((1, 1, 0)) #right turn
      #heading  2
      self.children[2].append((-1, 0, 2)) #straight
      self.children[2].append((-1, -1, 3)) #left turn
      self.children[2].append((-1, 1, 1)) #right turn
      #heading 3
      self.children[3].append((0, -1, 3)) #straight
      self.children[3].append((1, -1, 0)) #left turn
      self.children[3].append((-1, -1, 2)) #right turn

      self.predecessors = [[] for i in range(self.num_heading)]
      #heading 0
      self.predecessors[0].append((-1, 0, 0)) #straight
      self.predecessors[0].append((-1, 1, 3)) #left turn
      self.predecessors[0].append((-1, -1, 1)) #right turn
      #heading 1
      self.predecessors[1].append((0, -1, 1)) #straight
      self.predecessors[1].append((-1, -1, 0)) #left turn
      self.predecessors[1].append((1, -1, 2)) #right turn
      #heading 2
      self.predecessors[2].append((1, 0, 2)) #straight
      self.predecessors[2].append((1, -1, 1)) #left turn
      self.predecessors[2].append((1, 1, 3)) #right turn
      #heading 3
      self.predecessors[3].append((0, 1, 3)) #straight
      self.predecessors[3].append((1, 1, 2)) #left turn
      self.predecessors[3].append((-1, 1, 0)) #right turn
    
    elif self.connectivity == "dubins_turn_45":
      # self.resolution = 
      # self.children = np.array([])
      # self.predecessors =  np.array([])#define children and predecessors for 45 degree turns
      raise NotImplementedError

    elif self.connectivity == "reeds_schepp_turn_90":
      raise NotImplementedError; #define children and predecessors for reeds schepp 90 degree turns
    elif self.connectivity == "reeds_scehpp_turn_45":
      raise NotImplementedError; #least priority right now

    StateLattice.__init__(self, self.ndims, [self.x_lims[0], self.y_lims[0], 0], [self.x_lims[1], self.y_lims[1], 2.0*np.pi], self.resolution)
    #Precalculate successors and predecessors
    self.node_to_succs = dict()
    self.node_to_preds = dict()
    self.succ_costs = dict()
    self.pred_costs = dict()
    self.edge_precalc_done = False
    self.costs_precalc_done = False

  def node_to_state(self, node):
    """Convert a discrete node to a world state taking origin and rotation of lattice into account

    @param  node  - a tuple containing di screte (x,y,heading) coordinates 
    @return state - np array of state in continuous world coordinates 

    """
    xd = node[0]
    yd = node[1]
    pos = self.origin + np.array([self.resolution[0]*(cos(self.rotation)*xd - sin(self.rotation)*yd),
                                    self.resolution[1]*(sin(self.rotation)*xd + cos(self.rotation)*yd)])
    
    rot = angles.normalize_angle((node[2]*(2.0*np.pi))/(self.num_heading*1.0))

    state = (pos[0], pos[1], rot)
    return state
  
  def state_to_node(self, state):
    """Convert a continuous state(in world coordinates) to a discrete node in lattice coordinates)"""
    pos = np.array([state[0], state[1]])
    pos -= self.origin
    pos = np.divide(pos, self.resolution[0:2])
    #we need to rotate the state to lattice coordinates
    pos = np.array([ cos(self.rotation)*pos[0] + sin(self.rotation)*pos[1], 
                    -sin(self.rotation)*pos[0] + cos(self.rotation)*pos[1]])     
    pos = pos.astype(int, copy=False)
    rot = int(((angles.normalize_angle_positive(state[2])/2.0*np.pi)*self.num_heading)%self.num_heading)
    
    node = (pos[0], pos[1], rot)
    return node

  def get_edge(self, parent_node, child_node):
    """Given two discrete nodes, returns an edge(sequence of waypoints) connecting them. 
    The returned edge includes the start and end points
     
    @param : parent_node - discrete node which is the parent
    @param : child_node  - discrete node which is the child
    @return: edge        - list of waypoints(continuous) in world coordinates
    """
    parent_state = self.node_to_state(parent_node)
    child_state = self.node_to_state(child_node)
    return self.interpolate(parent_state, child_state)
    

  def get_successors(self, node):
    """Given a discrete node in the lattice, returns the child nodes (discrete) and corresponding
    edges (continuous states)

    The edges use the parameter path_resolution to interpolate between two nodes. 
    
    @param node    - node for which successors are to be queried
    @return succs  - a list of lists, where the first element is a child node (discrete) and second element is 
                     an edge connecting current node to child node (continuous states) 
    """
    succs = []
    children = self.children[node[2]]
    parent_state = self.node_to_state(node)
    for it in children:
      child_node = (node[0] + it[0], node[1] + it[1], it[2])
      child_state = self.node_to_state(child_node)
      edge = self.interpolate(parent_state, child_state)
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
    predecessors = self.predecessors[node[2]]
    child_state = self.node_to_state(node)
    for it in predecessors:
      parent_node = (node[0] + it[0], node[1] + it[1], it[2])
      parent_state = self.node_to_state(parent_node)
      edge = self.interpolate(child_state, parent_state)
      preds.append([parent_node, edge])
    return preds


  def interpolate(self, s1, s2):
    """Returns a sequence of states between two states 
    given a motion that connects the two states.

    We use a shooting method type approach to interpolate between the two states

    @param: s1         - parent state
    @param: s2         - child state
    @return edge       - sequence of states (including end points) 
    """
    step_size = self.path_resolution
    s1_n = (s1[0], s1[1], angles.normalize_angle_positive(s1[2]))
    s2_n = (s2[0], s2[1], angles.normalize_angle_positive(s2[2]))
    path = dubins.shortest_path(s1, s2, self.radius-0.01)
    edge, _ = path.sample_many(step_size)
    #Dubins returns edge in [0, 2pi], we got to normalize it to (-pi, pi] for our code
    normalized_edge = []
    
    for config in edge:
      new_config = (config[0], config[1], angles.normalize_angle(config[2]))
      normalized_edge.append(new_config)
    
    normalized_edge.append(tuple(s2))
    return normalized_edge
  
  def distance_bw_states(self, s1, s2):
    path = dubins.shortest_path(s1, s2, self.radius-0.01)
    return path.path_length()

  
  def enumerate_lattice(self):
    """For every discrete node in the lattice, we store a dictionary of [successor nodes, successor edges] and 
    [predecessor_node, predecessor_edges]"""
    node_to_succs = dict()
    node_to_preds = dict()
    start_node = self.state_to_node([self.x_lims[0], self.y_lims[0], 0])
    
    for h in range(self.num_heading):
      h = start_node[2] + h
      for i in range(self.num_cells[0]):
        x = start_node[0] + i
        for j in range(self.num_cells[1]):
          y = start_node[1] + j
          node = (x,y,h)
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

