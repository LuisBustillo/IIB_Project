#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys
import signal
import rospy
import yaml

import tf
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import scipy
import matplotlib.pyplot as plt

from geometry_msgs.msg import Twist, Point, Quaternion
# Occupancy grid.
from nav_msgs.msg import OccupancyGrid
# Position.
from tf import TransformListener
# Goal.
from geometry_msgs.msg import PoseStamped
# Path.
from nav_msgs.msg import Path
# For pose information.
from tf.transformations import euler_from_quaternion

# Import code rather than copy-pasting.
directory = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../python')
sys.path.insert(0, directory)
try:
    # import go_to_specific_point_on_map as Nav
    import measure_coverage as Area
  
except ImportError:
  raise ImportError('Unable to import functions. Make sure this file is in "{}"'.format(directory))

def handler(signum, frmae):
    raise Exception("Unable to reach pose :(")

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.15 / 2.



#TODO Edit GoToPose so that pose estimates come from slam/lidar not odometry
#TODO Make each new position a GoalPose object
#TODO Add remaining elements from run() function in cpp_navigation and rtt_navigation
# to follow_the_route.py

"""
class OccupancyGrid(object):
  def __init__(self, values, origin, resolution):
    self._original_values = values.copy()
    self._values = values.copy()
    # Inflate obstacles (using a convolution).
    inflated_grid = np.zeros_like(values)
    inflated_grid[values == OCCUPIED] = 1.
    w = 2 * int(ROBOT_RADIUS / resolution) + 1
    inflated_grid = scipy.signal.convolve2d(inflated_grid, np.ones((w, w)), mode='same')
    self._values[inflated_grid > 0.] = OCCUPIED
    self._origin = np.array(origin[:2], dtype=np.float32)
    self._origin -= resolution / 2.
    assert origin[YAW] == 0.
    self._resolution = resolution

  @property
  def values(self):
    return self._values

  @property
  def resolution(self):
    return self._resolution

  @property
  def origin(self):
    return self._origin

  def draw(self):
    plt.imshow(self._original_values.T, interpolation='none', origin='lower',
               extent=[self._origin[X],
                       self._origin[X] + self._values.shape[0] * self._resolution,
                       self._origin[Y],
                       self._origin[Y] + self._values.shape[1] * self._resolution])
    plt.set_cmap('gray_r')

  def get_index(self, position):
    idx = ((position - self._origin) / self._resolution).astype(np.int32)
    if len(idx.shape) == 2:
      idx[:, 0] = np.clip(idx[:, 0], 0, self._values.shape[0] - 1)
      idx[:, 1] = np.clip(idx[:, 1], 0, self._values.shape[1] - 1)
      return (idx[:, 0], idx[:, 1])
    idx[0] = np.clip(idx[0], 0, self._values.shape[0] - 1)
    idx[1] = np.clip(idx[1], 0, self._values.shape[1] - 1)
    return tuple(idx)

  def get_position(self, i, j):
    return np.array([i, j], dtype=np.float32) * self._resolution + self._origin

  def is_occupied(self, position):
    return self._values[self.get_index(position)] == OCCUPIED

  def is_free(self, position):
    return self._values[self.get_index(position)] == FREE
"""
class SLAM(object):
  def __init__(self):
    rospy.Subscriber('/map', OccupancyGrid, self.callback)
    self._tf = TransformListener()
    self._occupancy_grid = None
    self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
    
  def callback(self, msg):
    values = np.array(msg.data, dtype=np.int8).reshape((msg.info.width, msg.info.height))
    processed = np.empty_like(values)
    processed[:] = FREE
    processed[values < 0] = UNKNOWN
    processed[values > 50] = OCCUPIED
    processed = processed.T
    origin = [msg.info.origin.position[X], msg.info.origin.position[Y], 0.]
    resolution = msg.info.resolution
    self._occupancy_grid = OccupancyGrid(processed, origin, resolution)

  def update(self):
    # Get pose w.r.t. map.
    a = 'occupancy_grid'
    b = 'base_link'
    if self._tf.frameExists(a) and self._tf.frameExists(b):
      try:
        t = rospy.Time(0)
        position, orientation = self._tf.lookupTransform('/' + a, '/' + b, t)
        self._pose[X] = position[X]
        self._pose[Y] = position[Y]
        _, _, self._pose[YAW] = euler_from_quaternion(orientation)
      except Exception as e:
        print(e)
    else:
      print('Unable to find:', self._tf.frameExists(a), self._tf.frameExists(b))
    pass

  @property
  def ready(self):
    return self._occupancy_grid is not None and not np.isnan(self._pose[0])

  @property
  def pose(self):
    return self._pose

  @property
  def occupancy_grid(self):
    return self._occupancy_grid

#TODO Use this for localization of robot by choosing arrow location and direction through Rviz
class GoalPose(object):
  def __init__(self):
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)
    self._position = np.array([np.nan, np.nan], dtype=np.float32)

  def callback(self, msg):
    # The pose from RViz is with respect to the "map".
    self._position[X] = msg.pose.position[X]
    self._position[Y] = msg.pose.position[Y]
    print('Received new goal position:', self._position)

  @property
  def ready(self):
    return not np.isnan(self._position[0])

  @property
  def position(self):
    return self._position

# Main Navigation Code

# Point == Goal
def goto(slam, point, angle):
    (goal_x, goal_y, goal_z) = (0, 0, 0)
    goal_x = point['x']
    goal_y = point['y']
    goal_z = angle

    slam.update()

    #(position, rotation) = self.get_odom()
    position = slam.pose[:2]
    rotation = slam.pose[YAW]

    last_rotation = 0
    linear_speed = 0.1
    angular_speed = 1

    goal_distance = sqrt(pow(goal_x - position[X], 2) + pow(goal_y - position[Y], 2))
    distance = goal_distance
    old_distance = distance

    print("goal", goal_x, goal_y, goal_z)

    while distance > 0.05:
        if not slam.ready:
            rate_limiter.sleep()
            continue

        slam.update()

        #(position, rotation) = self.get_odom()
        position = slam.pose[:2]
        rotation = slam.pose[YAW]
        x_start = position[X]
        y_start = position[Y]

        path_angle = atan2(goal_y - y_start, goal_x- x_start)
        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
        z_dist = min(abs(rotation - path_angle), abs(2 * pi - abs(rotation - path_angle)))

        # detect and halt open loop behavior
        if distance > old_distance + 0.1:
            break

        # proportional control for linear speed
        move_cmd.linear.x = min(linear_speed * distance ** 1.1 + 0.1, linear_speed)

        # proportional control for heading
        if path_angle >= 0:
            if rotation <= path_angle and rotation >= path_angle - pi:
                move_cmd.angular.z = angular_speed * z_dist / (abs(move_cmd.linear.x) + 1)
            else:
                move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(move_cmd.linear.x) + 1)
        else:
            if rotation <= path_angle + pi and rotation > path_angle: 
                move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(move_cmd.linear.x) + 1)
            else:
                move_cmd.angular.z = angular_speed * z_dist / (abs(move_cmd.linear.x) + 1)

        old_distance = distance
            
        cmd_vel.publish(move_cmd)
            
    #(position, rotation) = self.get_odom()
    position = slam.pose[:2]
    rotation = slam.pose[YAW]

    if abs(goal_z) > pi / 2:
        while abs(rotation - goal_z) > 0.01:
            #(position, rotation) = self.get_odom()
            position = slam.pose[:2]
            rotation = slam.pose[YAW]

            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = min(1.5 * abs(rotation - goal_z), 1.5)
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -min(1.5 * abs(rotation - goal_z), 1.5)
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -min(1.5 * abs(rotation - goal_z), 1.5)
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = min(1.5 * abs(rotation - goal_z), 1.5)
            cmd_vel.publish(move_cmd)

    rospy.loginfo("point reached")
    cmd_vel.publish(Twist())

"""
def get_odom(self):
    try:
        (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])

"""


if __name__ == '__main__':
    
    print("follow_route running")
    rospy.loginfo("follow_route running")
    # Read information from yaml file
    with open("/home/luis/catkin_ws/src/IIB_Project/part2/python/route.yaml", 'r') as stream:
        dataMap = yaml.safe_load(stream)

    print("loaded")

    # m = Area.MeasureAreaCovered()
    try:
        # Initialize
        rospy.init_node('follow_route', anonymous=False)

        #rospy.on_shutdown(self.shutdown)
        cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        #position = Point()
        move_cmd = Twist()
        rate_limiter = rospy.Rate(1000)
        tf_listener = tf.TransformListener()
        odom_frame= 'odom'

        rate_limiter.sleep()
        try:
            tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(30.0))
            base_frame = 'base_footprint'
            print("base frame tf listener initialized")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                print("trying to transform")
                tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(30.0))
                base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                print("tfException")
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        print("all initialized")


        # Stop moving message.
        stop_msg = Twist()
        stop_msg.linear.x = 0.
        stop_msg.angular.z = 0.

        # Make sure the robot is stopped.
        i = 0
        while i < 10 and not rospy.is_shutdown():
            cmd_vel.publish(stop_msg)
            rate_limiter.sleep()
            i += 1

        slam = SLAM()

        for obj in dataMap:

            if rospy.is_shutdown():
                break
            name = obj['filename']
            print(obj)
            signal.signal(signal.SIGALRM, handler)
            signal.alarm(25)
            # Navigation
            try:
                rospy.loginfo("Go to %s pose", name[:-4])

                goto(slam, obj['position'], obj['rotation'])

                rospy.loginfo("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Reached %s pose", name[:-4])
            except Exception as exc:
                print(exc)


        # m.disp()


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")