#!/usr/bin/env python3

import rospy
from lab3_kinematics.msg import Traffic
import numpy as np

class TrafficNode(object):
    def __init__(self):
        # Set up traffic status publisher
        self.traffic_status_pub = rospy.Publisher("/traffic_status", Traffic)
        rospy.sleep(1)

    def run(self):
        # Once every 10 seconds
        rate = rospy.Rate(0.2)
        while (not rospy.is_shutdown()):
            # TODO: send traffic message
            traffic_msg = Traffic()
            traffic_msg.direction = "right" #"left"
            #print('traffic right')
            self.traffic_status_pub.publish(traffic_msg)
            rate.sleep()
            
            traffic_msg.direction = "left" #"left"
            #print('traffic left')
            self.traffic_status_pub.publish(traffic_msg)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('traffic_controller')
    traffic_node = TrafficNode()
    traffic_node.run()
