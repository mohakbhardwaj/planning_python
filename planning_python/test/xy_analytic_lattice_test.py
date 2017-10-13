#!/usr/bin/env python
import sys
sys.path.insert(0, "../..")
from planning_python.state_lattices.common_lattice.xy_analytic_lattice import XYAnalyticLattice

def construct_and_test_lattice(params):
  lattice = XYAnalyticLattice(params)
  #test node to state conversion
  node = (-10,-10)
  print('Converting node to state: ', node)
  state = lattice.node_to_state(node)
  print('State: ', state)
  #test state to node conversion
  print('Converting state back to node')
  node2 = lattice.state_to_node(state)
  print('Got back: ', node2)
  #Get id for the node
  print('Node id %d'%lattice.node_to_id(node2))
  #Get successors for node
  print('Successsors')
  print(lattice.get_successors(node2))



params = dict( x_lims          = [-100,100],
               y_lims          = [-100,100],
               connectivity    = 'four_connected',
               resolution      = [1,1], #resolution of grid spacing
               origin          = (50,50), #origin of lattice w.r.t  world
               rotation        = 0, #rotation of lattice about z-axis w.r.t world
               path_resolution = 0.5) #resolution to do collision checking at 

construct_and_test_lattice(params)