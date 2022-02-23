from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import matplotlib.pylab as plt
import matplotlib.patches as patches
import numpy as np
import os
import re

from numpy.lib.twodim_base import flipud
import scipy.signal
import yaml
import cv2
from stc import stc
from occupancy_grid import*

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.21 / 2.

START_POSE_1 = np.array([-1.0, -1.0, 0], dtype=np.float32)
START_POSE_2 = np.array([-0.95, 2.4, 0], dtype=np.float32)
START_POSE_3 = np.array([0.105, -0.55, -3.11], dtype=np.float32)
START_POSE_4 = np.array([-0.03, 0., 3.12], dtype=np.float32)

# Hepler Functions

def area_covered(occupancy_grid, positions, robot_radius, duplicates=False):
    """
    Estimates the percentage of an area the robot has covered from the robot dimensions and its x,y position
    Robot occupies a square of cells DxD large at any given time

    occupancy_grid: Occupancy_grid object 
    positions: list of tuples of x,y positions
    """

    #TODO convert list of x,y points into list of DxD grid occupied by robot
    #TODO remove duplicates from visited cell list

    free_area = 0
    for row in occupancy_grid._original_values:
      for element in row:
            if element == FREE:
                free_area += 1  

    total_area = (occupancy_grid.resolution**2)*free_area     # free area in m^2
    
    visited_map = occupancy_grid.values_trajectory.copy()
    visited_map_duplicates = occupancy_grid.values_trajectory.copy()

    for position in positions:
        point_idx = occupancy_grid.get_index(position)
        corners = [(position[0] - robot_radius, position[1] - robot_radius),
                  (position[0] - robot_radius, position[1] + robot_radius),
                  (position[0] + robot_radius, position[1] - robot_radius),
                  (position[0] + robot_radius, position[1] + robot_radius)]
        
        corner_indices = []
        for corner in corners:
            corner_indices.append(occupancy_grid.get_index(corner))
        
        for i in range(corner_indices[0][0], corner_indices[3][0] + 1):
            for j in range(corner_indices[0][1], corner_indices[3][1] + 1):
                if visited_map[i, j] == FREE:
                  visited_map[i, j] = -1
                  visited_map_duplicates[i, j] = -1
                
                else:
                  visited_map_duplicates[i, j] -= 1
    
    visited_area = 0
    for row in visited_map:
        for element in row:
            if element == -1:
                visited_area += 1  

    covered_area = (occupancy_grid.resolution**2)*visited_area     # visited area in m^2

    percentage_covered = 100*(covered_area/total_area)

    if duplicates:
      for i in range((occupancy_grid.values_trajectory).shape[0]):
        for j in range((occupancy_grid.values_trajectory).shape[1]):
          if visited_map_duplicates[i, j] < 0:
            occupancy_grid.values_trajectory[i, j] = visited_map_duplicates[i, j]

    else:
      occupancy_grid.values_trajectory[visited_map == -1] = -2

    return percentage_covered

def find_circle(node_a, node_b):
  def perpendicular(v):
    w = np.empty_like(v)
    w[X] = -v[Y]
    w[Y] = v[X]
    return w
  db = perpendicular(node_b.direction)
  dp = node_a.position - node_b.position
  t = np.dot(node_a.direction, db)
  if np.abs(t) < 1e-3:
    # By construction node_a and node_b should be far enough apart,
    # so they must be on opposite end of the circle.
    center = (node_b.position + node_a.position) / 2.
    radius = np.linalg.norm(center - node_b.position)
  else:
    radius = np.dot(node_a.direction, dp) / t
    center = radius * db + node_b.position
  return center, np.abs(radius)

def unit(a):
  a = np.array(a)
  mag = np.linalg.norm(a)
  if mag < 0.001:
    return None

  return a/mag

def find_dist_from_a_to_b(a, b):
  return np.linalg.norm(np.array(b)-np.array(a))

def rotation_from_a_to_b(a, b, current_yaw):

    disp = np.array(b) - np.array(a)
    theta = np.arctan2(disp[Y], disp[X])
    
    turn_angle = (theta - current_yaw)
    #TODO CHECK THE SIGN AND VALUE OF THE ROTATION
    if abs(turn_angle) < 0.001:
      turn_angle = 0.

    if abs(turn_angle) > np.pi:   # TODO CHECK EQUALITY
      turn_angle = -1*np.sign(turn_angle)*((2*np.pi) - abs(turn_angle))
    
    # CHECK
    #turn_angle = -turn_angle
    new_yaw = current_yaw + turn_angle

    return turn_angle, new_yaw

def draw_connections(path, linewidth=1.0, text=False, head_width=0.05, head_length=0.1, arrow_length=0.1):
  n = len(path)
  for node in range(n):
        current_node = path[node]
        if node == n-1:
            next_node = path[1]
        else:
            next_node = path[node+1]
        
        plt.plot([current_node[X], next_node[X]], [current_node[Y], next_node[Y]], 'b-', linewidth=linewidth)
        
        disp = np.array(next_node) - np.array(current_node)
        dir = unit(disp)
        if dir is not None:
          plt.arrow(current_node[X], current_node[Y], dir[0] * arrow_length, dir[1] * arrow_length,
              head_width=head_width, head_length=head_length, fc='k', ec='k')
        if text == True:
          plt.text(current_node[X], current_node[Y], str(node))

def draw_nodes(path, color, size=8):
  for node in path:
        plt.scatter(node[X], node[Y], s=size, marker='o', color=color, zorder=1000)

def draw_individual_node(point, color, size=10):
  plt.scatter(point[0], point[1], s=size, marker='o', color=color, zorder=1000)

def generate_yaml_path(path_points):
      with open('route.yaml', 'w') as f:
        index = 0
        for point in path_points:
          index += 1    
          print("- {filename: 'p%s', position: { x: %s, y: %s}, rotation: %s}" % (index, point[0], point[1], point[2]), file = f)
      print("Route file generated!")

# Defines a node of the graph.
class Node(object):
  def __init__(self, pose):
    self._pose = pose.copy()
    self._neighbors = []
    self._parent = None
    self._cost = 0.

  @property
  def pose(self):
    return self._pose

  def add_neighbor(self, node, weight):
    self._neighbors.append([node,weight])

  @property
  def parent(self):
    return self._parent

  @parent.setter
  def parent(self, node):
    self._parent = node

  @property
  def neighbors(self):
    return self._neighbors

  @property
  def position(self):
    return self._pose[:2]

  @property
  def yaw(self):
    return self._pose[YAW]
  
  @property
  def direction(self):
    return np.array([np.cos(self._pose[YAW]), np.sin(self._pose[YAW])], dtype=np.float32)

  @property
  def cost(self):
      return self._cost

  @cost.setter
  def cost(self, c):
    self._cost = c

def prm(start_pose, occupancy_grid, N=80):
    # Builds a Probabilistic Road-map (PRM) that is traversed by a DFS algorithm
    graph = []
    graph.append(start_pose)

    def sample_random_position(occupancy_grid):
        position = np.zeros(2, dtype=np.float32)

        # Sample a valid random position (do not sample the yaw).
        # The corresponding cell must be free in the occupancy grid.
  
        position = np.random.uniform(-1.8, 1.8, 2)		# Initialise the x,y position randomly for a SQUARE map
                                                        # Otherwise sample x and y separately 
        while not occupancy_grid.is_free(position):		# Resample if position is invalid
  	        position = np.random.uniform(-2, 2, 2)
  
        return position

    def sample_random_positions(N, occupancy_grid):
        while N > 0: 
            position = sample_random_position(occupancy_grid)
            position = np.append(position, 0)
            graph.append(position)
            for pos in graph:
                if (pos[0], pos[1]) == (position[0], position[1]):
                    pass

                elif np.linalg.norm(position[:2] - pos[:2]) < 0.25:
                    graph.pop()
                    N += 1
                    break
            N -= 1

    sample_random_positions(N, occupancy_grid)
    
    # Build Graph from sampled nodes
    for node in range(len(graph)):
        graph[node] = Node(graph[node])
    
    #For now orientation = 0 for all Nodes
    for node in graph:
        for node_neighbor in graph:
            dist = np.linalg.norm(node.position - node_neighbor.position)
            if node == node_neighbor:
                pass
            elif dist < 0.65:
                node.add_neighbor(node_neighbor, dist)

        # Plot Graph Connections        
        for neighbor in node.neighbors:
            pass
            #plt.plot([node.pose[X],neighbor[0].pose[X]], [node.pose[Y], neighbor[0].pose[Y]], 'k-')

    n = len(graph)
    # Array for node path
    visited = []

    def dfs(visited, graph, node):
        if node not in visited:
            visited.append(node)
            for neighbor, _ in node.neighbors:
                dfs(visited, graph, neighbor)

    dfs(visited, graph, graph[0])
    """
    # Plot PRM Nodes
    for node in graph:
        plt.scatter(node.pose[X], node.pose[Y], s=8, marker='o', color='red', zorder=1000)

    # Plot PRM Connections
    for node in range(n):
        current_node = visited[node]
        if node == n-1:
            next_node = visited[0]
        else:
            next_node = visited[node+1]
        
        plt.plot([current_node.pose[X], next_node.pose[X]], [current_node.pose[Y], next_node.pose[Y]], 'b-')
        plt.text(current_node.pose[X], current_node.pose[Y], str(node))
    
    """
    # Set the YAW for each node in graph
    def adjust_pose(current_node, next_node, occupancy_grid):

      next_position = next_node.position

      # Check whether there exists a simple path that links current_node.pose
      # to next_position. This function needs to return a new node that has
      # the same position as next_position and a valid yaw. The yaw is such that
      # there exists an arc of a circle that passes through current_node.pose and the
      # adjusted final pose. If no such arc exists (e.g., collision) return None.
      # Assume that the robot always goes forward.
      # Feel free to use the find_circle() function below.
  
      displacement = next_position - current_node.position
      theta = current_node.pose[YAW]
      direction = current_node.direction
      #direction = np.array([np.cos(theta), np.sin(theta)])
  
      beta = np.arctan2(displacement[Y], displacement[X]) - np.arctan2(direction[Y], direction[X])	
      alpha = np.arctan2(displacement[Y], displacement[X])
  
      next_node.pose[YAW] = alpha + beta
  
      centre, radius = find_circle(current_node, next_node)
  
      arc_angle_1 = -np.pi/2 + theta 			        # angle from centre of circle to start position
      arc_angle_2 = -np.pi/2 + next_node.pose[YAW] 	# angle from centre of circle to end position

      if arc_angle_1 >= arc_angle_2:
  	    arc_angle_1, arc_angle_2 = arc_angle_2, arc_angle_1

      dtheta = 0.005
  
      for angle in np.arange(arc_angle_1, arc_angle_2, dtheta):

  	    pos = centre + [radius*np.cos(angle), radius*np.sin(angle)]

  	    if not occupancy_grid.is_free(pos):
  		    return None
  				

    for node in range(n):
        current_node = visited[node]
        if node == n-1:
          break
        else:
          next_node = visited[node+1]

        count = 2
        while adjust_pose(current_node, next_node, occupancy_grid) is None:
          
          idx_pop = visited.index(next_node)
          visited.pop(idx_pop)
          visited.append(next_node)

          if node + count >= n-1:
            break
          else:
            next_node = visited[node+count] 
            count += 1

    for node in visited:
      print(node.pose)

    return visited

def cpp(start_pose, occupancy_grid, start_indices=[155, 155], end_indices=[245,245], scale=6):
    
    unpad = np.zeros_like(occupancy_grid.values)

    # Defining obstacles and free space
    unpad[occupancy_grid.values == OCCUPIED] = -1
    unpad[occupancy_grid.values == UNKNOWN] = -1
    
    #TODO ADD PADDING TO OBSTACLES

    # Slicing grid defined by .pgm file to contain environment area ONLY
    sub_cells = unpad[start_indices[0]:end_indices[0], start_indices[1]:end_indices[1]]

    sub_cells = np.array(sub_cells, dtype='uint8')

    # Calclualting the size of the Mega-cell grid
    dsize = (int((end_indices[0]- start_indices[0])/scale), int((end_indices[1]- start_indices[1])/scale))
    
    #TODO  TEST WHICH INTERPOLATION METHOD IS THE MOST CONSERVATIVE
    cpp_grid = cv2.resize(sub_cells, dsize=dsize, interpolation=cv2.INTER_NEAREST)
    
    for i in range(len(cpp_grid[0, :])):
        for j in range(len(cpp_grid[:, 0])):
          if cpp_grid[i, j] != 0:
            cpp_grid[i, j] = -1
          

    #TODO WRITE MAPPING FUNCTIONS (MERGE CELLS INTO SUB-CELLS && MERGE SUB-CELLS INTO MEGA-CELLS)

    def index_mapping():
      pass

    def inv_index_mapping():
      pass

    
    #start_indices = index_mapping(start_position)
    
    # Path consisiting of points expressed as indices in simplified grid
    path = stc(cpp_grid)
    
    n = len(path)
    """
    plt.matshow(cpp_grid.T, None)
    # Start Point
    plt.scatter(path[0][0], path[0][1], s=10, marker='o', color='green', zorder=1000)
    # End Point
    plt.scatter(path[-1][0], path[-1][1], s=10, marker='o', color='red', zorder=1000)

    # Drawing Connections
    draw_connections(path, head_width=0.2, head_length=0.2, arrow_length=0.5)
    """
    def convert_path(path, start_indices=start_indices, end_indices=end_indices,  scale=scale):
      new_path = []
      for point in path:
        point = np.array(point)
        point = np.array(start_indices) + point*scale
        new_path.append(point)

      return new_path
    
    # Path consisiting of points expressed as indices in original map
    converted_path = convert_path(path)
    
    # Path consisiting of points expressed as coordinates in original map
    coord_path = []
    for point in converted_path:
      coord_path.append(occupancy_grid.get_position(point[0], point[1]))

    
    #TODO USE GET_INDEX METHOD FOR STARTING POSITION
    start_dist = 999
    start_point = np.zeros(2, dtype=np.float32)
    start_index = 0

    for index, point in enumerate(coord_path):
      if find_dist_from_a_to_b(start_pose[:2], point) < start_dist:
        
        start_dist = find_dist_from_a_to_b(start_pose[:2], point)
        start_point = point
        start_index = index

    coord_path = [*coord_path[start_index:],*coord_path[:start_index]]    
    
    # SORT OUT YAW FROM START POSE

    initial_yaw = start_pose[YAW]
    coord_path.insert(0, start_pose[:2])

    def instruction_list(path, yaw):
      # Path where each node is stored in the format required by YAML file
      yaml_path = []
      
      # list of strings --> each string is an instruction
      instructions = []     

      n = len(path)

      for node in range(n):
        current_node = path[node]
        if node == n-1:
          next_node = path[1]
        else:
          next_node = path[node+1]

        rot, yaw = rotation_from_a_to_b(current_node, next_node, yaw)

        yaml_node = np.append(current_node, rot)
        yaml_path.append(yaml_node)

        if rot != 0:
          instructions.append("rt" + str(np.rad2deg(rot)))

        dist = find_dist_from_a_to_b(current_node, next_node)
        instructions.append("fd" + str(dist))
        instructions.append("STOP")

      return instructions, yaml_path

    #TODO OVERLAY PATH ONTO ORIGINAL MAP USING POSITIONS OR INDICES

    # Draw Envrionmnet
    draw_nodes(coord_path[1:-1], color='red')
    # Starting position of robot
    draw_individual_node(coord_path[0], color='violet')
    # Start Node of CPP path
    draw_individual_node(coord_path[1], color='green')
    # End Node
    draw_individual_node(coord_path[-1], color='yellow')
    draw_connections(coord_path, text=False, head_width=0.03, head_length=0.05, arrow_length=0.04)
    
    return instruction_list(coord_path, initial_yaw)

def draw_solution(points):
  ax = plt.gca()

  # Function to draw path between points
  def draw_path(u, v, arrow_length=.1, color=(.8, .8, .8), lw=1):
    du = u.direction
    plt.arrow(u.pose[X], u.pose[Y], du[0] * arrow_length, du[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    dv = v.direction
    plt.arrow(v.pose[X], v.pose[Y], dv[0] * arrow_length, dv[1] * arrow_length,
              head_width=.05, head_length=.1, fc=color, ec=color)
    center, radius = find_circle(u, v)
    du = u.position - center
    theta1 = np.arctan2(du[1], du[0])
    dv = v.position - center
    theta2 = np.arctan2(dv[1], dv[0])
    # Check if the arc goes clockwise.
    if np.cross(u.direction, du).item() > 0.:
      theta1, theta2 = theta2, theta1
    ax.add_patch(patches.Arc(center, radius * 2., radius * 2.,
                             theta1=theta1 / np.pi * 180., theta2=theta2 / np.pi * 180.,
                             color=color, lw=lw))

  for point in points[1:]:

    plt.scatter(point.pose[X], point.pose[Y], s=10, marker='o', color='red')

  #Draw start node
  plt.scatter(points[0].pose[X], points[0].pose[Y], s=10, marker='o', color='green', zorder=1000)
  
  for node in range(len(points)):
    current_node = points[node]

    if node == len(points) - 1:
        break
    else:
        next_node = points[node+1]

    # Draw final path.
    draw_path(current_node, next_node)


if __name__ == '__main__':
  
  # Plot environment.
  
  fig, ax = plt.subplots()
  occupancy_grid.draw()
  
  """
  # MAP 1 - 4M20 
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-.5 - 2., 2. + .5])
  plt.ylim([-.5 - 2., 2. + .5])
  
  inst, yaml = cpp(START_POSE_1, occupancy_grid, start_indices=[300, 300], end_indices=[500,500], scale=13) # scale = 6
  generate_yaml_path(yaml)
  
  # MAP 2 - Turtlebot World

  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-1.0 - 2., 2. + .75])
  plt.ylim([-.7 - 2., 2. + .7])

  inst, yaml = cpp(START_POSE_2, occupancy_grid, start_indices=[160, 160], end_indices=[400, 400], scale=15)
  generate_yaml_path(yaml)
  """
  # MAP 3 - IIB Project
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-1., 1.])
  plt.ylim([-1., 1.])
  
  print(occupancy_grid.get_index([-0.77, -0.7]))
  print(occupancy_grid.get_index([0.71, 0.75]))

  inst, yaml = cpp(START_POSE_3, occupancy_grid, start_indices=[900, 900], end_indices=[1100, 1100], scale=10)
  # start_indices=[370, 370], end_indices=[430, 430], scale=10 for res of 0.025
  generate_yaml_path(yaml)
    
  """
  # MAP 4 - IIB Test
  
  plt.axis('equal')
  plt.xlabel('x')
  plt.ylabel('y')
  plt.xlim([-1., 1.])
  plt.ylim([-1., 1.])
  
  print(occupancy_grid.get_index([-0.77, -0.7]))
  print(occupancy_grid.get_index([0.71, 0.75]))

  inst, yaml = cpp(START_POSE_4, occupancy_grid, start_indices=[920, 920], end_indices=[1080, 1080], scale=10)
  generate_yaml_path(yaml)
  """
  plt.show()
