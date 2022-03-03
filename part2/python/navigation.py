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
import time
import subprocess
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
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

# Constants for PID Controller
kp_distance = 0.03       # 1
ki_distance = 0.01
kd_distance = 0.05

kp_angle = 0.5          # 1
ki_angle = 0.03
kd_angle = 0.05

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

class GoToPose():
    def __init__(self):
        #rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.slam = SLAM()
        self.positions = []
        # self.sdr = RtlSdrTcpClient(hostname='192.168.228.210', port=55366)
        # SDR.configure_device(self.sdr, center_freq=914.6e6)

        self.r.sleep()

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def goto(self, point, angle):
        (goal_x, goal_y, goal_z) = (0, 0, 0)
        goal_x = point['x']
        goal_y = point['y']
        goal_z = angle

        self.slam.update()

        #(self.position, rotation) = self.get_odom()
        self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
        rotation = self.slam.pose[YAW]

        self.positions.append((self.position.x, self.position.y))

        last_rotation = 0
        linear_speed = 0.03     #kp_distance
        angular_speed = 0.3     #kp_angular

        goal_distance = sqrt(pow(goal_x - self.position.x, 2) + pow(goal_y - self.position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0
       

        while distance > 0.03:

            if not self.slam.ready:
              self.r.sleep()
              continue

            self.slam.update()

            #(self.position, rotation) = self.get_odom()
            self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
            rotation = self.slam.pose[YAW]

            #self.positions.append((self.position.x, self.position.y))

            x_start = self.position.x
            y_start = self.position.y

            #path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:

                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle

                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle

            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation

            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation


            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = 0.8*(kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance)

            control_signal_angle = 0.4*(kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle)

            self.move_cmd.angular.z = (control_signal_angle) - rotation
            #self.move_cmd.linear.x = min(linear_speed * distance, 0.1)
            self.move_cmd.linear.x = min(control_signal_distance, 0.03)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 0.3)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -0.3)

            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            previous_distance = distance
            previous_angle = path_angle
            total_angle = total_angle + path_angle
            total_distance = total_distance + distance

        while abs(rotation - goal_z) > 0.01:
            #(self.position, rotation) = self.get_odom()
            self.slam.update()
            self.position = Point(self.slam.pose[X], self.slam.pose[Y], 0)
            rotation = self.slam.pose[YAW]

            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.3
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.3
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.3
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.3
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        rospy.loginfo("Pose Reached!")

        self.positions.append((self.position.x, self.position.y))
        
        self.cmd_vel.publish(Twist())

        #Measure and calculate RF signal power
        # samples = SDR.receive_samples(self.sdr)
        # max_power, _ = SDR.get_power_from_PSD(samples, self.sdr, freq=915.1e6, plot=False)
        # (self.data).append(np.array([self.position.x, self.position.y, max_power]))

    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view == 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

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