#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3


class DrivingSquare(object):
    """ This node commands the Turtlebot3 robot to drive a square path """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('driving_in_a_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel',
                                                  Twist,
                                                  queue_size=10)
        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

    def run(self):
        # Create a default twist msg (all values 0).
        twist = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0.1))
        # twist.linear.x = 0.1; twist.angular.z = 0.1
        # TODO
        # setup the Twist message we want to send

        # Publish msg to cmd_vel.
        self.robot_movement_pub.publish(twist)


if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DrivingSquare()
    node.run()