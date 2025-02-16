#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math

from random import randint, random
from likelihood_field import LikelihoodField


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion(
        [p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w])[2])

    return yaw


class Particle:

    def __init__(self, pose, w):
        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w


class ParticleFilter:

    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # initialize our map
        self.init_map = False
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud",
                                             PoseArray,
                                             queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose",
                                                  PoseStamped,
                                                  queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        # wait until the map is ready
        while (not self.init_map):
            rospy.sleep(1)
        self.likelihood_field = LikelihoodField(self.map)

        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True

    def get_map(self, data):
        self.map = data
        self.init_map = True

    def initialize_particle_cloud(self):

        resolution = self.map.info.resolution
        map_width = int(resolution * self.map.info.width)
        map_height = int(resolution * self.map.info.height)
        map_x_offset = self.map.info.origin.position.x
        map_y_offset = self.map.info.origin.position.y

        inside_house = []
        """
        Iterate over every point on the map (by the map's cell dimension),
        and add a particle if that location is unoccupied.
        """
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                if self.map.data[j * self.map.info.width + i] == 0:
                    inside_house.append((i, j))

        for i in range(self.num_particles):
            index = np.random.randint(len(inside_house))
            location = inside_house[index]
            """
            x and y are set to the location on the surface of the map,
            determined by the above loop.
            """
            x = resolution * location[0] + map_x_offset
            y = resolution * location[1] + map_y_offset
            """
            The z-axis is set to be random, as this would correspond to
            some direction the robot would be facing, level to the ground.
            """
            x_rot, y_rot, z_rot = 0, 0, random_sample() * 2 * math.pi
            q = quaternion_from_euler(x_rot, y_rot, z_rot)
            self.particle_cloud.append(
                Particle(Pose(Point(x, y, 0), Quaternion(*q)), w=1))

        self.normalize_particles()

        self.publish_particle_cloud()

    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        sum_weights = 0

        for p in self.particle_cloud:
            sum_weights += p.w

        for p in self.particle_cloud:
            p.w /= sum_weights

    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(),
                                                  frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)

    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(),
                                                    frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)

    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not (self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not (self.tf_listener.canTransform(
                self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to bas e_footprint transform was updated)
        self.tf_listener.waitForTransform(self.base_frame,
                                          self.odom_frame, data.header.stamp,
                                          rospy.Duration(0.5))
        if not (self.tf_listener.canTransform(
                self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0), frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=data.header.stamp,
                                      frame_id=self.base_frame),
                        pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return

        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold
                    or np.abs(curr_y - old_y) > self.lin_mvmt_threshold
                    or np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):
                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose

    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        # TODO

        return


