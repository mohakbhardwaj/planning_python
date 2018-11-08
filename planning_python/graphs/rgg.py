import numpy as np
import networkx as nx



class RGG():
  def __init__(self, ndims, nnodes, r, pos=None, lower_limits=[], upper_limits=[], path_resolution=[]):
    self.ndims = ndims
    self.nnodes = nnodes
    self.r = r
    self.pos = pos
    self.graph = nx.random_geometric_graph(nnodes, r, ndims, pos)
    self.lower_limits = np.array(lower_limits)
    self.upper_limts = np.array(upper_limits)

  def get_successors(self, node):
    # node = self.state_to_node(state)
    succs = self.graph.get_neighbors(node)
    return succs

  def node_to_state(self, node):
    state = []
    return state

  def state_to_node(self, state):
    return node





if __name__=="__main__":
  import random
  n = 4
  pos = {i: (random.gauss(0, 2), random.gauss(0, 2)) for i in range(n)}
  # print pos
  rgg = RGG(2, n, 1, pos)
  for i in n:

  print rgg.graph.node[1]