#!/usr/bin/env python3

# TOPICS:
#   cmd_vel: publish to, used for setting robot velocity
#   scan   : subscribing, where the wall is

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist


class WallFollower(object):
    """ This node command the robot to drive alongside the wall at an approximately fixed distance """

    def __init__(self):
        # Start rospy node.
        rospy.init_node("wall_follower")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.robot_movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        self.twist = Twist()

    def process_scan(self, data):
        # Determine closeness to wall by looking at scan data from the robot, 
        # set linear velocity and angular based on that information, and publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.
        # The first entry in the ranges list corresponds with what's directly
        #   in front of the robot.
        # example: if data.ranges[0] < 10: do_something 

        # TODO

        # Publish msg to cmd_vel.
        right = min(data.ranges[0:60])
        wall = max(data.ranges[0:45])
        head = data.ranges[0]

       
        if  head < 0.18: #avoid stuck in  the corner
        	self.twist.linear.x = -0.2
        	rospy.sleep(2)
        	
        else:
        	self.twist.linear.x = 0.2
        	self.twist.angular.z = 0.0
        	
        	if right > 0.8 :
        		self.twist.linear.x = 0.2
        		self.twist.angular.z = 0.0
        	else :
        		self.twist.linear.x = 0.2
        		self.twist.angular.z = 1.2
        		
        			
        
        self.robot_movement_pub.publish(self.twist)
          
        
        
       	
        	

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()
