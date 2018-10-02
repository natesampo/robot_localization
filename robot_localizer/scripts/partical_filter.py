#!/usr/bin/env python
from __future__ import division
import tty
import select
import sys
import termios
import rospy
import os
import thread
import signal
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from nav_msgs.msg import Odometry
import Xlib.display as display
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import numpy as np
import occupancy_field as O_F
#import helper_functions

"""
The Plan:
- The Occupancy_field file contains maps of rooms and a method that gets the distance to the nearest point given an xy coordinates
- We place a bunch of random points in the room and get an xy distance for each of them
- We then use the lidar to find the distance to the nearest obstacle
- Then, we look at the distances and decide which one is more probable
- We then move the robot and all the points (at this point the little points turn into 360 points each with a point going off in each direction)
- Then, the robot's new closest distantce value is compared to the new points
- This is repeated until we find the point. ?

"""




def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

def make_more_points(particle,transformation):
    """this function takes a particle object and a list of tuples. The tuples contain the trainsformation steps
    (action,)

     """

# Creating the neato's class
class Neato(object):
    def _init_(self):
            rospy.init_node("localize", disable_signals=True)
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.getOdom)
            self.velocityPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=5)

    def getOdom(self, msg):
        self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

    def current_closest object (self):
        # uses the lidar to find the closest object

    def rand_translate(self):
        #method decides to randomly translate the robot

#creating a class for a certain particle
class particle(object):
    def _init_(self):
        self.x = 0
        self.y = 0
        self.history = []
