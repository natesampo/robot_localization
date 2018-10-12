#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseArray, Pose
import numpy as np
import math
from std_msgs.msg import Header
from helper_functions import TFHelper
from occupancy_field import OccupancyField
from particle_manager import ParticleManager, Particle
from sensor_manager import SensorManager

def getDistance(a1, a2, b1, b2):
    return math.sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2))

class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        self.particle_publisher = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.particle_manager = ParticleManager()
        self.sensor_manager = SensorManager()
        self.particle_manager.init_particles(self.occupancy_field)
        self.scanDistance = 0.2
        self.scanAngle = 0.5
        self.moved = (0, 0)

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.particle_manager.init_particles(self.occupancy_field)
        poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
        for particle in self.particle_manager.particles:
            poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
        #    print(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
        self.particle_publisher.publish(poseArray)

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()): # while ros is not shutdown...
            #do not understand what is going on here....
            if not self.sensor_manager.newLaserScan and ((getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]) > self.scanDistance) or self.transform_helper.angle_diff(math.radians(self.sensor_manager.pose[2]), math.radians(self.sensor_manager.lastScan[2])) > self.scanAngle):
                # ^^ if its not a new laser scan and either the distance between the last scan interval and the curent pose value is greater than 0.2 or the angle difference is greater than the scan angle threshold
                # basically, if we turn or move and havent taken a scan for awhile, take a new scan
                self.sensor_manager.newLaserScan = True #take a new scan
                self.moved = (getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]), math.degrees(self.transform_helper.angle_diff(self.sensor_manager.pose[2], self.sensor_manager.lastScan[2])))
                # we have moved this much?
            elif self.sensor_manager.newLaserScan: # if we want to take a new laser scan
                #print(len(self.particle_manager.particles))
                while self.sensor_manager.newLaserScan: #while we are taking the scan, wait.
                    continue
                self.particle_manager.transform_particles(self.moved[0], self.moved[1])
                self.particle_manager.update_probabilities(self.sensor_manager.minRange, self.occupancy_field, self.sensor_manager.closestAngles)
                self.particle_manager.trim_particles()
                self.particle_manager.generate_particles()
                poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
                for particle in self.particle_manager.particles:
                    poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
                self.particle_publisher.publish(poseArray)

            self.transform_helper.send_last_map_to_odom_transform()

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
