#!/usr/bin/env python
import dubins
import numpy as np
from planning_python.utils import angles


class HeuristicFunction(object):
  def __init__(self):
    return None

  def get_heuristic(self, state, goal):
      return NotImplementedError


class EuclideanHeuristicNoAng(HeuristicFunction):
  """Computes euclidean distance between two states.

  Useful when states do not contain angular terms
  """
  def __init__(self):
    super(EuclideanHeuristicNoAng, self).__init__()

  def get_heuristic(self, state, goal):
    return np.linalg.norm(goal-state)


class ManhattanHeuristicNoAng(HeuristicFunction):
  """Computes manhattan distance between two states.

  Useful when states do not contain angular terms
  """
  def __init__(self):
    super(ManhattanHeuristicNoAng, self).__init__()

  def get_heuristic(self, state, goal):
    return np.sum(np.abs(goal-state))


class OctileHeuristicNoAng(HeuristicFunction):
  """Computes octile distance between two states.

  Useful when states do not contain angular terms
  """
  def __init__(self):
    super(OctileHeuristicNoAng, self).__init__()

  def get_heuristic(self, state, goal):
    temp = np.abs(np.array(state) - np.array(goal))
    return max(temp) + 0.414 * min(temp)

class EuclideanHeuristicAng(HeuristicFunction):
  """Computes euclidean distance between two states.

  Useful when states contain angular terms
  """
  def __init__(self):
    super(EuclideanHeuristicAng, self).__init__()

  def get_heuristic(self, state, goal):
    return np.linalg.norm(goal[:-1]-state[:-1])


class ManhattanHeuristicAng(HeuristicFunction):
  """Computes manhattan distance between two states.

  Useful when states contain angular terms
  """
  def __init__(self):
    super(ManhattanHeuristicAng, self).__init__()

  def get_heuristic(self, state, goal):
    return np.sum(np.abs(goal[:-1]-state[:-1]))


class OctileHeuristicAng(HeuristicFunction):
  """Computes octile distance between two states.

  Useful when states contain angular terms
  """
  def __init__(self):
    super(OctileHeuristicAng, self).__init__()

  def get_heuristic(self, state, goal):
    temp = np.abs(np.array(state[:-1]) - np.array(goal[:-1]))
    return max(temp) + 0.414 * min(temp)

class DubinsHeuristic(HeuristicFunction):
  """Computes dubins distnace between two configurations : (x1, y1, th1) and (x2, y2, th2)

  Configurations must be of the form (x,y,heading). 
  Initialize with turning radius as a parameter
  """
  def __init__(self, turning_radius=None):
    assert turning_radius is not None, "Please enter turning radius parameter"
    self.turning_radius = turning_radius

  def get_heuristic(self, state, goal):
    assert len(state) == 3, "state must be of form (x,y,heading)"
    assert len(goal) == 3, "goal must be of form (x,y,heading)"
    s = (state[0], state[1], angles.normalize_angle_positive(state[2]))
    g = (goal[0], goal[1], angles.normalize_angle_positive(goal[2]))
    path = dubins.shortest_path(s, g, self.turning_radius-0.01)
    return path.path_length()

