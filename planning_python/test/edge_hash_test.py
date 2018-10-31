"""Test the numbre of hash collisions when using a dictionary to store edges of a lattice. """
#!/usr/bin/env python
import sys
sys.path.insert(0, "../..")
import numpy as np
import time
from planning_python.state_lattices.common_lattice   import XYAnalyticLattice

#Step1: Set some problem parameters
x_lims = [0, 201] # low(inclusive), upper(exclusive) extents of world in x-axis
y_lims = [0, 201] # low(inclusive), upper(exclusive) extents of world in y-axis
start = (10, 10)    #start state(world coordinates)
goal = (199, 199)  #goal state(world coordinates)
visualize = False


#Step 2: Create a lattice
lattice_params = dict()
lattice_params['x_lims']          = x_lims   # Usefule to calculate number of cells in lattice 
lattice_params['y_lims']          = y_lims   # Useful to calculate number of cells in lattice
lattice_params['resolution']      = [1, 1]   # Useful to calculate number of cells in lattice + conversion from discrete to continuous space and vice-versa
lattice_params['origin']          = start    # Used for conversion from discrete to continuous and vice-versa. 
lattice_params['rotation']        = 0        # Can rotate lattice with respect to world
lattice_params['connectivity']    = 'eight_connected' #Lattice connectivity (can be four or eight connected for xylattice)
lattice_params['path_resolution'] = 1         #Resolution for defining edges and doing collision checking (in meters)

l = XYAnalyticLattice(lattice_params)

#For every node we get all the successor and predecessor edges.
print('Enumerating Lattice')
node_to_succs, node_to_preds = l.enumerate_lattice()

h_vals = []
num_collisions = 0
hash_time = 0
num_edges = 0
print('Calculating hash collisions')
print ('Number of nodes = %d'%len(node_to_succs.keys()))
for key, value in node_to_succs.iteritems():
  for mov, edge in value:
    start_t = time.time()
    h_val = hash(edge)
    hash_time += time.time() - start_t #Add time taken per hash to total hashing time 
    num_edges += 1

    if h_val in h_vals:
      num_collisions += 1
    else:
      h_vals.append(h_val)

avg_hash_time = (hash_time*1.0)/(num_edges*1.0)
print('Number of hash collisions %d'%num_collisions)
print('Total hashing time = %f, Num edges = %d, Average hashing time = %f'%(hash_time, num_edges, avg_hash_time))


