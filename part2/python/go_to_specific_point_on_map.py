#!/usr/bin/env python

# Turtlebot3 pointop_key navigation

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import scipy
import matplotlib.pyplot as plt
import yaml
import signal

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

# For SDR antenna
from pylab import *
from rtlsdr import *
import sdr as SDR

# Constants used for indexing.
X = 0
Y = 1
YAW = 2

# Constants for occupancy grid.
FREE = 0
UNKNOWN = 1
OCCUPIED = 2

ROBOT_RADIUS = 0.21 / 2.
pose_offset = [0., 0., 0.]   # [-0.03, 0., 3.12] from SLAM

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
    origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.]
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
    self._position[X] = msg.pose.position.x
    self._position[Y] = msg.pose.position.y
    print('Received new goal position:', self._position)

  @property
  def ready(self):
    return not np.isnan(self._position[0])

  @property
  def position(self):
    return self._position

# Main Navigation Code

class GoToPose():
    def __init__(self):
        # rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(1000)
        self.tf_listener = tf.TransformListener()
        self.odom_frame= 'odom'
        self.slam = SLAM()
        self.positions = []
        self.data = []
        self.sdr = RtlSdrTcpClient(hostname='192.168.228.210', port=55366)
        SDR.configure_device(self.sdr, center_freq=914.6e6)
        self.offset = pose_offset

        self.r.sleep()
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(30.0))
            self.base_frame = 'base_footprint'
            print("base frame tf listener initialized")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                print("trying to transform")
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(30.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                print("tfException")
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        print("all initialized")

    def goto(self, point, angle):
        (goal_x, goal_y, goal_z) = (0, 0, 0)
        goal_x = point['x']
        goal_y = point['y']
        goal_z = angle

        self.slam.update()

        #(self.position, rotation) = self.get_odom()
        #self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
        #rotation = self.slam.pose[YAW]
        self.position = Point(self.slam.pose[X] - self.offset[X], self.slam.pose[Y] - self.offset[Y], 0)
        rotation = self.slam.pose[YAW] - self.offset[YAW]

        self.positions.append((self.position.x, self.position.y))

        last_rotation = 0
        linear_speed = 0.03
        angular_speed = 0.3

        goal_distance = sqrt(pow(goal_x - self.position.x, 2) + pow(goal_y - self.position.y, 2))
        distance = goal_distance
        old_distance = distance

        print("goal", goal_x, goal_y, goal_z)

        while distance > 0.025:
            if not self.slam.ready:
              self.r.sleep()
              continue

            self.slam.update()

            #(self.position, rotation) = self.get_odom()
            #self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
            #rotation = self.slam.pose[YAW]
            self.position = Point(self.slam.pose[X] - self.offset[X], self.slam.pose[Y] - self.offset[Y], 0)
            rotation = self.slam.pose[YAW] - self.offset[YAW]

            #self.positions.append((self.position.x, self.position.y))

            x_start = self.position.x
            y_start = self.position.y

            path_angle = atan2(goal_y - y_start, goal_x- x_start)
            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            z_dist = min(abs(rotation - path_angle), abs(2 * pi - abs(rotation - path_angle)))

            # detect and halt open loop behavior
            if distance > old_distance + 0.1:
                break

            # proportional control for linear speed
            self.move_cmd.linear.x = min(linear_speed * distance ** 1.1 + 0.01, linear_speed)

            # proportional control for heading
            if path_angle >= 0:
                if rotation <= path_angle and rotation >= path_angle - pi:
                    self.move_cmd.angular.z = angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 0.65) # + 0.5
                else:
                    self.move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 0.65)
            else:
                if rotation <= path_angle + pi and rotation > path_angle: 
                    self.move_cmd.angular.z = -1 * angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 0.65)
                else:
                    self.move_cmd.angular.z = angular_speed * z_dist / (abs(self.move_cmd.linear.x) + 0.65)

            old_distance = distance
            
            self.cmd_vel.publish(self.move_cmd)
        
        self.slam.update()

        #(self.position, rotation) = self.get_odom()
        #self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
        #rotation = self.slam.pose[YAW]
        self.position = Point(self.slam.pose[X] - self.offset[X], self.slam.pose[Y] - self.offset[Y], 0)
        rotation = self.slam.pose[YAW] - self.offset[YAW]

        self.positions.append((self.position.x, self.position.y))

        if abs(goal_z) > pi / 2:
            while abs(rotation - goal_z) > 0.01:
                #(self.position, rotation) = self.get_odom()
                self.slam.update()
                #self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
                #rotation = self.slam.pose[YAW]
                self.position = Point(self.slam.pose[X] - self.offset[X], self.slam.pose[Y] - self.offset[Y], 0)
                rotation = self.slam.pose[YAW] - self.offset[YAW]

                if goal_z >= 0:
                    if rotation <= goal_z and rotation >= goal_z - pi:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = min(1. * abs(rotation - goal_z), 0.3)
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -min(1. * abs(rotation - goal_z), 0.3)
                else:
                    if rotation <= goal_z + pi and rotation > goal_z:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = -min(1. * abs(rotation - goal_z), 0.3)
                    else:
                        self.move_cmd.linear.x = 0.00
                        self.move_cmd.angular.z = min(1. * abs(rotation - goal_z), 0.3)
                self.cmd_vel.publish(self.move_cmd)

        # print(self.position.x, self.position.y, rotation)
        self.slam.update()

        self.positions.append((self.position.x, self.position.y))

        rospy.loginfo("point reached")

        self.cmd_vel.publish(Twist())

        #Measure and calculate RF signal power
        samples = SDR.receive_samples(self.sdr)
        max_power, _ = SDR.get_power_from_PSD(samples, self.sdr, freq=915.1e6, plot=False)
        (self.data).append(np.array([self.position.x, self.position.y, max_power]))

        self.r.sleep()

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def generate_path(self):
      with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/path.yaml', 'w') as f:
          index = 0
          for point in self.positions:
            index += 1    
            print("- {filename: 'p%s', position: { x: %s, y: %s} }" % (index, point[0], point[1]), file = f)
          print(" Path File Generated")

    def generate_rf_data(self):
      with open('/home/luis/catkin_ws/src/IIB_Project/part2/ros/data.yaml', 'w') as f:
          index = 0
          for point in self.data:
            index += 1    
            print("- {filename: 'p%s', position: { x: %s, y: %s}, power: %s}" % (index, point[0], point[1], point[2]), file = f)
          print(" RF Data File Generated")

    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

