#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.probability = 0

    def transform(self, d, theta):
        self.x += math.cos(math.radians(self.theta))*d
        self.y += math.sin(math.radians(self.theta))*d
        self.theta += theta % 360

class ParticleManager(object):
    """ The class that manages the Particles. Contains a list of all of the particles and operates on them
    """
    def __init__(self):
        self.angleResolution = 18
        self.particles = []
        self.keepRate = 0.3
        self.totalParticles = 250
        self.initDeviationXY = 0.3
        self.initDeviationTheta = math.pi/3

    def generate_particles(self, pose):
        """This method generates all particles within our accepted deviation of the estimated pose"""
        for i in range():
            da

    def trim_particles(self):
        """ Trims particles down to keepRate * total particles using random weighted sampling. Here we are keeping 30% of particles """


    def transform_particles(self, d, theta):
        """ Transforms all particles to match the robots transform """
        for particle in self.particles:
            particle.transform(d, theta)

    def update_probabilities(self, closestScan, OF):
        """ Updates the probability for every particle that it is the correct robot position """
        for particle in self.particles:
            particle.probability = abs(closestScan, OF.get_closest_obstacle_distance(particle.x, particle.y)[0])
