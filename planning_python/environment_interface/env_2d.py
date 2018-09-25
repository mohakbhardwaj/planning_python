#!/usr/bin/env python
""" @package environment_interface
Loads an environment file from a database and returns a 2D
occupancy grid.

Inputs : file_name, x y resolution (meters to pixel conversion)
Outputs:  - 2d occupancy grid of the environment
          - ability to check states in collision
"""
import numpy as np
import math
from time import sleep
import matplotlib.pyplot as plt
import matplotlib.image as mpimage
from scipy import ndimage
from planning_python.utils import helpers

class Env2D():
  def __init__(self):
    self.plot_initialized = False
    self.distance_transform_available = False
    self.image = None


  def initialize(self, envfile, params):
    """Initialize environment from file with given params

      @param envfile - full path of the environment file
      @param params  - dict containing relevant parameters
                           {x_lims: [lb, ub] in x coordinate (meters),
                            y_lims: [lb, ub] in y coordinate (meters)}
      The world origin will always be assumed to be at (0,0) with z-axis pointing outwards
      towards right
    """
    try:
      self.image = plt.imread(envfile)
      if len(self.image.shape) > 2:
        self.image = helpers.rgb2gray(self.image)
    except IOError:
      print("File doesn't exist. Please use correct naming convention for database eg. 0.png, 1.png .. and so on. You gave, %s"%(envfile))
    self.x_lims = params['x_lims']
    self.y_lims = params['y_lims']

    self.x_res  = (self.x_lims[1] - self.x_lims[0])/((self.image.shape[1]-1)*1.)
    self.y_res  = (self.y_lims[1] - self.y_lims[0])/((self.image.shape[0]-1)*1.)

    orig_pix_x = math.floor(0 - self.x_lims[0]/self.x_res) #x coordinate of origin in pixel space
    orig_pix_y = math.floor(0 - self.y_lims[0]/self.y_res) #y coordinate of origin in pixel space
    self.orig_pix = (orig_pix_x, orig_pix_y)
    self.distance_transform_available = False
   

  def collision_free(self, state):
    """ Check if a state (continuous values) is in collision or not.

      @param state - tuple of (x,y) or (x,y,th) values in world frame
      @return 1 - free
              0 - collision
    """
    try:
      pix_x, pix_y = self.to_image_coordinates(state)
      return round(self.image[pix_y][pix_x])
    except IndexError:
      print("Out of bounds, ", state, pix_x, pix_y)
      return 0
    

  def in_limits(self, state):
    """Filters a state to lie between the environment limits

    @param state - input state
    @return 1 - in limits
          0 - not in limits
    """
    pix_x, pix_y = self.to_image_coordinates(state)    
    if 0 <= pix_x < self.image.shape[1] and 0<= pix_y < self.image.shape[0] and self.x_lims[0] <= state[0] < self.x_lims[1] and self.y_lims[0] <= state[1] < self.y_lims[1]:
      return True
    return False


  def is_state_valid(self, state):
    """Checks if state is valid.

    For a state to be valid it must be within environment bounds and not in collision
    @param state - input state
    @return 1 - valid state
            0 - invalid state
    """
    if self.in_limits(state) and self.collision_free(state):
      return 1
    return 0
  
  def is_edge_valid(self, edge):
    """Takes as input an  edge(sequence of states) and checks if the entire edge is valid or not
    @param edge - list of states including start state and end state
    @return 1 - valid edge
            0 - invalid edge
            first_coll_state - None if edge valid, else first state on edge that is in collision
    """
    valid_edge = True
    first_coll_state = None
    for state in edge:
      if not self.in_limits(state):
        valid_edge = False
        break
      if not self.collision_free(state):
        valid_edge = False
        first_coll_state = state
        break
    return valid_edge, first_coll_state


  def to_image_coordinates(self, state):
    """Helper function that returns pixel coordinates for a state in
    continuous coordinates

    @param  - state in continuous world coordinates
    @return - state in pixel coordinates """
    pix_x = int(self.orig_pix[0] + math.floor(state[0]/self.x_res))
    pix_y = int(self.image.shape[1]-1 - (self.orig_pix[1] + math.floor(state[1]/self.y_res)))
    return (pix_x,pix_y)
  
  def to_world_coordinates(self, pix):
    """Helper function that returns world coordinates for a pixel

    @param  - state in continuous world coordinates
    @return - state in pixel coordinates """
    world_x = (pix[0] - self.orig_pix[0])*self.x_res 
    world_y = (pix[1] - self.orig_pix[0])*self.y_res   
    return (world_x, world_y)

  def get_env_lims(self):
    return self.x_lims, self.y_lims
  
  def calculate_distance_transform(self, sampling=[1,1]):
    if not self.distance_transform_available:
      sampling_pix = np.divide(np.array(sampling), np.array([self.x_res, self.y_res]))
      self.edt= ndimage.distance_transform_edt(self.image, sampling)
      self.norm_edt = self.edt/np.amax(self.edt)
      self.dy, self.dx = np.gradient(self.edt) #get gradients as well 
      self.dy_norm = self.dy/np.amax(self.dy)
      self.dx_norm = self.dx/np.amax(self.dx)
      self.distance_transform_available = True
      print('Calculated Distance Transform')
  
  def get_obstacle_distance(self, state, norm=True):
    pix_x, pix_y = self.to_image_coordinates(state)
    if norm:
      d_obs = self.norm_edt[pix_y][pix_x] #distance to obstacle    
      obs_dx = self.dx_norm[pix_y][pix_x] #gradient in x direction 
      obs_dy = self.dy_norm[pix_y][pix_x] #gradient in y direction     
    else:
      d_obs = self.edt[pix_y][pix_x] #distance to obstacle    
      obs_dx = self.dx[pix_y][pix_x] #gradient in x direction 
      obs_dy = self.dy[pix_y][pix_x] #gradient in y direction 
    return d_obs, obs_dx, obs_dy

  def initialize_plot(self, start, goal, grid_res=None, plot_grid=False):
    # if not self.plot_initialized:
    self.figure, self.axes = plt.subplots()
    self.axes.set_xlim(self.x_lims)
    self.axes.set_ylim(self.y_lims)
    if plot_grid and grid_res:
      self.axes.set_xticks(np.arange(self.x_lims[0], self.x_lims[1], grid_res[0]))
      self.axes.set_yticks(np.arange(self.y_lims[0], self.y_lims[1], grid_res[1]))
      self.axes.grid(which='both')
    self.figure.show()
    self.visualize_environment()
    self.line, = self.axes.plot([],[])
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 
    self.plot_state(start, 'red')
    self.plot_state(goal, 'green')
    self.figure.canvas.draw()
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 
    self.plot_initialized = True


  def reset_plot(self, start, goal, grid_res=None):
    if self.plot_initialized:
      plt.close(self.figure) 
      self.initialize_plot(start, goal, grid_res)

  def visualize_environment(self):
    # if not self.plot_initialized:
    self.axes.imshow(self.image, extent = (self.x_lims[0], self.x_lims[1], self.y_lims[0], self.x_lims[1]), cmap='gnuplot')


  def plot_edge(self, edge, linestyle='solid', color='blue', linewidth=2):
    x_list = []
    y_list = []
    for s in edge:
      x_list.append(s[0])
      y_list.append(s[1])
    self.figure.canvas.restore_region(self.background)
    self.line.set_xdata(x_list)
    self.line.set_ydata(y_list)
    self.line.set_linestyle(linestyle)
    self.line.set_linewidth(linewidth)
    self.line.set_color(color)
    self.axes.draw_artist(self.line)
    self.figure.canvas.blit(self.axes.bbox)
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox) 

  def plot_edges(self, edges,linestyle='solid', color='blue', linewidth=2):
    """Helper function that simply calls plot_edge for each edge"""
    for edge in edges:
      self.plot_edge(edge, linestyle, color, linewidth)

  def plot_state(self, state, color = 'red'):
    """Plot a single state on the environment"""
    # self.figure.canvas.restore_region(self.background)
    self.axes.plot(state[0], state[1], marker='o', markersize=3, color = color)
    self.figure.canvas.blit(self.axes.bbox)
    self.background = self.figure.canvas.copy_from_bbox(self.axes.bbox)
  
  def plot_path(self, path, linestyle='solid', color='blue', linewidth=2):
    flat_path = [item for sublist in path for item in sublist]
    self.plot_edge(flat_path, linestyle, color, linewidth)

  def close_plot(self):
    if self.plot_initialized:
      plt.close(self.figure)
      self.plot_initialized = False

  def clear(self):
    if self.plot_initialized:
      plt.close(self.figure)
      self.plot_initialized = False
    if self.distance_transform_available:
      del self.edt[:], self.norm_edt[:], self.dy[:], self.dx[:], self.dy_norm[:], self.dx_norm[:]
      self.distance_transform_available = False
    self.image = None