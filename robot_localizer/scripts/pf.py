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
    #a^2 + b^2 = c^2
    return math.sqrt((a1 - b1)*(a1 - b1) + (a2 - b2)*(a2 - b2))

class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')
        self.particle_publisher = rospy.Publisher("particlecloud", PoseArray, queue_size=10)
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()
        self.particle_manager = ParticleManager()
        self.sensor_manager = SensorManager()
        self.particle_manager.init_particles(self.occupancy_field)
        self.scanDistance = 0.2
        self.scanAngle = 0.5
        self.moved = (0, 0)

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):

            #Create a pose array with all of the current particles for easy viewing in Rviz
            poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
            for particle in self.particle_manager.particles:
                poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))

            #Publish the pose array of the current particles
            self.particle_publisher.publish(poseArray)

            #If we are not already looking for a new laser scan but have moved a moderate distance, set the flag to look for a new laser scan
            if not self.sensor_manager.newLaserScan and ((getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]) > self.scanDistance) or self.transform_helper.angle_diff(math.radians(self.sensor_manager.pose[2]), math.radians(self.sensor_manager.lastScan[2])) > self.scanAngle):

                #Set the flag to look for a new laser scan and mark where we last took laser scan data so we know how far we can move before taking another
                self.sensor_manager.newLaserScan = True
                self.moved = (getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]), math.degrees(self.transform_helper.angle_diff(self.sensor_manager.pose[2], self.sensor_manager.lastScan[2])))

            #Here, we are currently looking for a new laser scan and haven't moved sufficient distance to warrant a new one
            elif self.sensor_manager.newLaserScan:

                #While we are looking for a new laser scan, wait for the data to come in so that we don't process the particles on old data
                while self.sensor_manager.newLaserScan:
                    continue

                #Tranform all particles to match the transform the robot has done since the last laser scan and particle trimming
                self.particle_manager.transform_particles(self.moved[0], self.moved[1])

                #Update the particle probabilities so we know how likely each particle is based on current laser scan data
                self.particle_manager.update_probabilities(self.sensor_manager.minRange, self.occupancy_field, self.sensor_manager.closestAngles)

                #Do a random weighted selection of all of the current particles to choose particles to be passed on to the next round of laser scan data
                self.particle_manager.trim_particles()

                #Generate new particles around the kept particles with some added noise
                self.particle_manager.generate_particles()

                #Create and publish the pose array with the new particles for viewing in Rviz
                poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
                for particle in self.particle_manager.particles:
                    poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
                self.particle_publisher.publish(poseArray)

            self.transform_helper.send_last_map_to_odom_transform()

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
