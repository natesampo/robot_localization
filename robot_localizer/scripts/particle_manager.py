#!/usr/bin/env python

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

import random
import math
import numpy as np

from helper_functions import TFHelper
from occupancy_field import OccupancyField

class Particle(object):
	'''
	Particles represent potential positions of the robot, and are defined in terms of an x, y coordinate location and a theta angle
	'''
    def __init__(self, x, y, theta):
    	'''
    	Particle is generated with an x, y, and theta value to determine the pose of the potential robot position
		keepRange is the probability of the particle being chosen to be kept for the next round

		Every round, 500 random numbers will be generated, and the 500 particles whose keepRange values surround those numbers will be kept for the next round
		example: keepRange = (500, 510) ... Random number = 505, particle is kept ... Random number = 490, particle is not kept
		Done in this way so that we do not have to normalize the entire list of particle probabilities
    	'''
        self.x = x
        self.y = y
        self.theta = theta
        self.keepRange = (0, 0)

    def transform(self, d, theta):
    	'''
    	Called for every particle after the robot has moved a certain distance
    	Translates the particle the same amount as the robot has translated
    	'''
        self.theta += theta % 360
        self.x += math.cos(math.radians(self.theta))*d
        self.y += math.sin(math.radians(self.theta))*d

class ParticleManager(object):
    """ The class that manages the Particles. Contains a list of all of the particles and operates on them
    """
    def __init__(self):
        self.angleResolution = 18
        self.particles = []
        self.keepRate = 1/10
        self.totalParticles = 5000
        self.deviationXY = 0.1
        self.deviationTheta = 5
        self.totalParticleProbability = 0
        self.maxDistanceFromWall = 1
        self.maxProbability = 300

    def init_particles(self, OF):
        """
        Generates the initial set of particles 
        """
        while len(self.particles) < self.totalParticles:
        	#Create a particle within the map, taking nothing else into consideration. Generate the full 5000 particles randomly across the map
            tempParticle = Particle(random.randrange(1000*round(OF.map.info.width*OF.map.info.resolution))/1000, random.randrange(round(1000*OF.map.info.height*OF.map.info.resolution/3))/1000, random.randrange(0, 360))
            
            #Append the newly generated particle to the master particle list
            self.particles.append(tempParticle)

    def generate_particles(self):
        """ This method generates all particles within our accepted deviation of the current particles """

      	#This is called after the particles are trimmed, so we must re-generate the rest of the particles from the number of kept particles to the number of total particles
        keptParticleNumber = len(self.particles)
        for i in range(keptParticleNumber, self.totalParticles):

        	#Every new particle we generate will be a copy of the previously kept particle with some added noise
        	#This means that each particle that was kept will get 10 'copies' of itself in the next round
            copyParticle = self.particles[i % keptParticleNumber]

            #Generate the deviation/noise in each axis
            xdev = random.randrange(0, self.deviationXY*200)/100 - self.deviationXY
            ydev = random.randrange(0, self.deviationXY*200)/100 - self.deviationXY
            tdev = random.randrange(0, self.deviationTheta*200)/100 - self.deviationTheta

            #Put the new 'clone' into the master particle list
            self.particles.append(Particle(copyParticle.x + xdev, copyParticle.y + ydev, copyParticle.theta + tdev))

    def trim_particles(self):
        """ Trims particles down to keepRate * total particles using random weighted sampling. Here we are keeping 10% of particles """
        if (self.totalParticleProbability != 0):
            keptParticles = []

            #While we don't have enough kept particles for the next round, keep chosing more to keep
            while len(keptParticles) < self.totalParticles*self.keepRate:

            	#Generate a random number up to the total particle probability to choose which particle to keep
                keep = random.randrange(0, math.ceil(self.totalParticleProbability))

                #Check all particles to see which particle this number falls into
                for particle in self.particles:

                	#If this random number fell into the range for this particle, keep this particle
                    if keep >= particle.keepRange[0] and keep < particle.keepRange[1]:
                        keptParticles.append(Particle(particle.x, particle.y, particle.theta))
                        break

            self.particles = keptParticles

    def transform_particles(self, d, theta):
        """ Transforms all particles to match the robots transform """
        for particle in self.particles:
            particle.transform(d, theta)

    def update_probabilities(self, closestScan, OF, closestAngles):
        """ Updates the probability for every particle that it is the correct robot position """
        self.totalParticleProbability = 0
        for particle in self.particles:

        	#closestDist is how close the closest obstacle is in the occupancy field for this particle
            closestDist = OF.get_closest_obstacle_distance(particle.x, particle.y)

            #if closestDist is NaN or is an unreasonable distance away from a wall, the particle will have 0 probability
            if closestDist != closestDist or closestDist > self.maxDistanceFromWall:
                tempProbability = 0
            else:
            	#Award points for the camparison of the lidar scan to the particle's closest distance to a wall
                tempProbability = self.maxProbability - 100*abs(closestScan - closestDist)
                tempAngle = 0

                #For several angles around the robot, check how close the nearest wall is in the that direction
                for angle in closestAngles:
                    if angle != 0.0 and angle != np.inf:
                        closestDist = OF.get_closest_obstacle_distance(particle.x + math.cos(math.radians((particle.theta + tempAngle) % 360))*angle, particle.y + math.sin(math.radians((particle.theta + tempAngle) % 360))*angle)
                        if closestDist == closestDist and closestDist <= self.maxDistanceFromWall/4:

                        	#Award points for the lidar scan data and occupancy field data being similar at this angle
                            tempProbability += self.maxProbability*2 - 100*abs(angle - closestDist)

                    tempAngle += self.angleResolution

            #keep the total probability of all of the particles so that we can generate a random number from 0 to the total probability to choose particles to keep in the future
            #This is the same as normalizing, as all particles have thier probability out of the total probability of being chosen
            particle.keepRange = (self.totalParticleProbability, self.totalParticleProbability + tempProbability)
            self.totalParticleProbability += tempProbability
