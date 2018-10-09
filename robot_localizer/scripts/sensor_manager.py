#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
import numpy as np

from helper_functions import TFHelper
from occupancy_field import OccupancyField

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

class SensorManager:
    def __init__(self):
        self.newLaserScan = false
        self.laserScan = []
        self.pose = (0, 0, 0)
        self.minRange = np.inf
        self.lastScan = self.pose
        rospy.Subscriber("/scan", LaserScan, self.getLaserScan)
        rospy.Subscriber("/odom", Odometry, self.getOdometry)

    def getLaserScan(self, msg):
        if self.newLaserScan:
            self.minRange = np.inf
            for i in len(msg.ranges):
                tempRange = msg.ranges[i]
                if tempRange != 0.0 and tempRange < self.minRange:
                    self.minRange = tempRange

            self.lastScan = self.pose
            self.newLaserScan = false

    def getOdometry(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)
