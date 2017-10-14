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
    # print node[2]
    children = self.children[node[2]]
    parent_state = self.node_to_state(node)
    for it in children:
      child_node = (node[0] + it[0], node[1] + it[1], it[2])
      child_state = self.node_to_state(child_node)
      edge = self.interpolate(parent_state, child_state)
      succs.append([child_node, edge])
      # print len(edge)
      # print([child_node, edge])
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
    edge, _ = dubins.path_sample(s1, s2, self.radius, step_size)

    #Dubins returns edge in [0, 2pi], we got to normalize it to (-pi, pi] for our code
    normalized_edge = []
    
    for config in edge:
      new_config = (config[0], config[1], angles.normalize_angle(config[2]))
      normalized_edge.append(new_config)
    
    normalized_edge.append(tuple(s2))
    return normalized_edge
  
  def distance_bw_states(self, s1, s2):
    return dubins.path_length(s1, s2, self.radius)

  


  # def in_bounds(self, config):
  #   # config = self.node_id_to_configuration(node_id)
  #   return np.all(self.lower_limits <= config) and np.all(config < self.upper_limits)
  
  # # def check_collision(self, node_id):
  # #   return node_id in self.walls
  
  # def get_successors(self, node_id):
  #   curr_config = self.node_id_to_configuration(node_id)
  #   # print curr_config
  #   results = []
  #   obs_neighbors = []
  #   motions = []
  #   temp = []
  #   int_points = []
    
  #   for i in xrange(self.max_movement_id):
  #     append_pose = True
  #     curvature, length = self.movements[i]
  #     new_config = CurveSegment.end_pose(curr_config, curvature, length)
  #     nidx = self.configuration_to_node_id(new_config)
      
  #     #Collision check along the motion primitive
  #     points = CurveSegment.segment_points(curr_config, curvature, length, 0.1)
  #     # print "new_config", new_config
  #     # print "points ", points
  #     for point in points:
  #       pidx = self.configuration_to_node_id(point)
  #       if self.check_collision(pidx):
  #         if self.in_bounds(point):
  #           obs_neighbors.append(pidx)
  #         append_pose = False
  #         break
        
  #       if not self.in_bounds(point):
  #         append_pose = False
  #         break

  #     if append_pose and self.in_bounds(new_config):
  #       results.append(nidx)
  #       motions.append(i)
    
        
  #   return results, motions, obs_neighbors #int_points

  # # Helper functions.
  # def distance(self, p, q):
  #   return sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

  # def states_close(self, nid_1, nid_2):
  #   """Checks if two poses (x, y, heading) are the same within a tolerance."""
  #   p = self.node_id_to_configuration(nid_1)
  #   q = self.node_id_to_configuration(nid_2)
  #   d_angle = abs((p[2]-q[2]+pi) % (2.0*pi) - pi)
  #   return d_angle <= radians(self.deg_tol*1.0) and self.distance(p, q) <= self.dist_tol*1.0
  #   return self.distance(p, q) <= self.dist_tol*1.0

  # def cost(self, from_nid, to_nid, movement):
  #   return self.weights.get(to_nid, 1)

  # def set_rectangular_obstacles(self, obs_corner_configs, obstacle_cost):
  #   for corners in obs_corner_configs:
  #     #All possible thetas are in collision for given x,y
  #     th = self.lower_limits[2]
  #     while th <= self.upper_limits[2]:
  #       x1, y1, th1 = self.configuration_to_grid_coord(np.array(corners[0:2]+[th]))
  #       x2, y2, _ = self.configuration_to_grid_coord(np.array(corners[2:] +[th])) 
  
  #       for i in xrange(x1, x2+1):
  #         for j in xrange(y1, y2+1):
  #           self.walls.add(self.grid_coord_to_node_id(np.array([i, j, th1])))
  #       th += self.resolution[2]

  # def get_interim_points(self, from_id, movement):
  #   curr_config = self.node_id_to_configuration(from_id)
  #   curvature, length = movement
  #   points = CurveSegment.segment_points(curr_config, curvature, length, 0.1)
  #   return [self.configuration_to_node_id(x) for x in points]

  # def get_motion_length(self, motion):
  #   curvature, length = motion
  #   return length


