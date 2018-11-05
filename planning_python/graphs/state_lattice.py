#!/usr/bin/env python
"""Defines a generic state lattice class from which other state lattices should inherit

    Additionally, this class defines hashing functions for discrete lattices and some visualization functions
    Author: Mohak Bhardwaj
    Date: August 24, 2017"""

import numpy as np

class StateLattice():
  def __init__(self, ndims, lower_limits = [], upper_limits = [], resolution = []):
    self.lower_limits = lower_limits
    self.upper_limits = upper_limits
    self.resolution = resolution
    self.ndims = ndims
    self.num_cells = self.ndims * [0]
    self.num_cells = [int(np.ceil((upper_limits[idx] - lower_limits[idx]) / resolution[idx])) for idx in
                    range(self.ndims)] #Total number of cells in each dimension
    self.total_cells = reduce(lambda x, y: x * y, self.num_cells) #Total number of cells in entire graph
        
    #Calculate and store hash value for all cells

    #Calculate and store inverse hash value for all cells

  def node_to_id(self, node):
    """Hash graph node to integer"""
    id = 0
    for i in xrange(self.ndims):
      mul = 1
      for j in xrange(self.ndims-i-1):
        mul = mul*self.num_cells[j]
      id = id + node[self.ndims-i-1]*mul
      id = int(id)
    return id

  def id_to_node(self, id):
    """Retrieve graph node corresponding to the hashed value"""
    node = np.array([0] * self.ndims)
    for i in reversed(xrange(self.ndims)):
      w = 1
      for j in xrange(i):
        w *= self.num_cells[j]
      q = np.floor(id / w)
      node[i] = q
      id = id % w
    return node.astype(int)

  def get_id_from_node(self, node):
    """Return hash value for a node from a lookup table """
    return self.node_to_id(node)

  def get_node_from_id(self, id):
    """Return graph node for a given hash value"""
    return self.id_to_node(id)
  

