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

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should
        be in radians) the difference is always based on the closest
        rotation from angle a to angle b.
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = self.angle_normalize(a)
    b = self.angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2

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
        self.scanAngle = 30
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
<<<<<<< Updated upstream
=======
        #    print(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
>>>>>>> Stashed changes
        self.particle_publisher.publish(poseArray)

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()): # while ros is not shutdown...
            print('why wont this print')
            #do not understand what is going on here....
            if not self.sensor_manager.newLaserScan and ((getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]) > self.scanDistance) or (math.degrees(angle_diff(math.radians(self.sensor_manager.pose[2]), math.radians(self.sensor_manager.lastScan[2]))) > self.scanAngle)):
                # ^^ if its not a new laser scan and either the distance between the last scan interval and the curent pose value is greater than 0.2 or the angle difference is greater than the scan angle threshold
                # basically, if we turn or move and havent taken a scan for awhile, take a new scan
                print("encountered ifnot")
                self.sensor_manager.newLaserScan = True #take a new scan
                self.moved = (getDistance(self.sensor_manager.lastScan[0], self.sensor_manager.lastScan[1], self.sensor_manager.pose[0], self.sensor_manager.pose[1]), math.degrees(angle_diff(math.radians(self.sensor_manager.pose[2]), math.radians(self.sensor_manager.lastScan[2]))))
                # we have moved this much?
            elif self.sensor_manager.newLaserScan: # if we want to take a new laser scan
                print("encounterd elif")
                #print(len(self.particle_manager.particles))
                while self.sensor_manager.newLaserScan: #while we are taking the scan, wait.
                    continue
                self.particle_manager.transform_particles(self.moved[0], self.moved[1]))
                self.particle_manager.update_probabilities(self.sensor_manager.minRange, self.occupancy_field)
                print(len(self.particle_manager.particles)
                self.particle_manager.trim_particles()
                self.particle_manager.generate_particles()
                print("generated some Particles")
                poseArray = PoseArray(header = Header(seq = 10, stamp = rospy.get_rostime(), frame_id = 'map'))
                for particle in self.particle_manager.particles:
<<<<<<< Updated upstream
                    poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particle.x, particle.y, particle.theta))
=======
                    poseArray.poses.append(self.transform_helper.convert_xy_and_theta_to_pose(particl.x, particle.y, particle.theta))
                print(len(self.particle_manager.particles))
>>>>>>> Stashed changes
                self.particle_publisher.publish(poseArray)

            self.transform_helper.send_last_map_to_odom_transform()

if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
