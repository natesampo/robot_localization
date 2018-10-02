#!/usr/bin/env python

""" This is the starter code for the robot localization project """

from __future__ import print_function, division
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose

from helper_functions import TFHelper
from occupancy_field import OccupancyField


def make_more_points(old_particle, transformation):
    """This function takes a point particle object and a transformation list. The list contains tupples one for
     each transformations. The first element in the tuple is the distance driven straight. the z translation is the turning """
     point_children = []
     for angle in range(0,342,18):
         new_child = Particle(old_particle.x, old_particle.y, (old_particle.theta+angle)%360)
         new_child.history = old_particle.history.append((old_particle.x,old_particle.y,old_particle.theta))
         for i in transformation:
             new_child.transform(transformation[i][0],transformation[i][1])
         point_children.append(new_child)
    return point_children


class Particle():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.history = []

    def transform(self, d, theta):
        self.x += math.cos(math.radians(self.theta))*d
        self.y += math.sin(math.radians(self.theta))*d
        self.theta += theta % 360

class ParticleFilter(object):
    """ The class that represents a Particle Filter ROS Node
    """
    def __init__(self):
        rospy.init_node('pf')

        # pose_listener responds to selection of a new approximate robot
        # location (for instance using rviz)
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        # publisher for the particle cloud for visualizing in rviz.
        self.particle_pub = rospy.Publisher("particlecloud",
                                            PoseArray,
                                            queue_size=10)

        # create instances of two helper objects that are provided to you
        # as part of the project
        self.occupancy_field = OccupancyField()
        self.transform_helper = TFHelper()

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        xy_theta = \
            self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)

        # TODO this should be deleted before posting
        self.transform_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        # initialize your particle filter based on the xy_theta tuple

    def run(self):
        r = rospy.Rate(5)

        while not(rospy.is_shutdown()):
            # in the main loop all we do is continuously broadcast the latest
            # map to odom transform
            self.transform_helper.send_last_map_to_odom_transform()
            r.sleep()


if __name__ == '__main__':
    n = ParticleFilter()
    n.run()
