#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

import random
import math

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.keepRange = (0, 0)

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
        self.keepRate = 1/3
        self.totalParticles = 240
        self.deviationXY = 0.1
        self.deviationTheta = 5
        self.totalParticleProbability = 0

    def init_particles(self, OF):
        """ Generates the initial set of particles """
        while len(self.particles) < self.totalParticles:
            self.particles.append(Particle(random.randrange(0, OF.map.info.width), random.randrange(OF.map.info.height), random.randrange(0, 360)))

    def generate_particles(self):
        """ This method generates all particles within our accepted deviation of the current particles """
        keptParticleNumber = len(self.particles)
        for i in range(keptParticleNumber, self.totalParticles):
            copyParticle = self.particles[i % keptParticleNumber]
            xdev = random.randrange(0, self.deviationXY*200)/100 - self.deviationXY
            ydev = random.randrange(0, self.deviationXY*200)/100 - self.deviationXY
            tdev = random.randrange(0, self.deviationTheta*200)/100 - self.deviationTheta
            self.particles.append(Particle(copyParticle.x + xdev, copyParticle.y + ydev, copyParticle.theta + tdev))

    def trim_particles(self):
        """ Trims particles down to keepRate * total particles using random weighted sampling. Here we are keeping 30% of particles """
        if (self.totalParticleProbability != 0):
            keptParticles = []
            while len(keptParticles) < self.totalParticles*self.keepRate:
                keep = random.randrange(0, math.ceil(self.totalParticleProbability))
                for particle in self.particles:
                    if keep >= particle.keepRange[0] and keep < particle.keepRange[1]:
                        keptParticles.append(Particle(particle.x, particle.y, particle.theta))
                        break

            self.particles = keptParticles

    def transform_particles(self, d, theta):
        """ Transforms all particles to match the robots transform """
        for particle in self.particles:
            particle.transform(d, theta)

    def update_probabilities(self, closestScan, OF):
        """ Updates the probability for every particle that it is the correct robot position """
        self.totalParticleProbability = 0
        for particle in self.particles:
            closestDist = OF.get_closest_obstacle_distance(particle.x, particle.y)
            if closestDist != closestDist:
                tempProbability = 0
            else:
                tempProbability = 1000 - 100*abs(closestScan - closestDist)

            particle.keepRange = (self.totalParticleProbability, self.totalParticleProbability + tempProbability)
            self.totalParticleProbability += tempProbability
