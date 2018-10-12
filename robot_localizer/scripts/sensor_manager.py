#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np

from helper_functions import TFHelper
from occupancy_field import OccupancyField
import math

import tf.transformations as t
from tf import TransformListener
from tf import TransformBroadcaster

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = t.euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

class SensorManager:
    def __init__(self):
        self.newLaserScan = True
        self.laserScan = []
        self.pose = (0, 0, 0)
        self.minRange = np.inf
        self.frontRange = np.inf
        self.lastScan = self.pose
        self.closestAngles = []
        rospy.Subscriber("/odom", Odometry, self.getOdometry)
        rospy.Subscriber("/scan", LaserScan, self.getLaserScan)

    def getLaserScan(self, msg):
        if self.newLaserScan:
            self.closestAngles = []
            self.minRange = np.inf
            self.frontRange = np.inf
            for tempRange in msg.ranges:
                if tempRange != 0.0 and tempRange < self.minRange:
                    self.minRange = tempRange

            for i in range(0, 360, 18):
                self.closestAngles.append(msg.ranges[i])

            self.lastScan = self.pose
            self.newLaserScan = False

    def getOdometry(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
