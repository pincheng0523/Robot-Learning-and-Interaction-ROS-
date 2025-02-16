#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

class Manipulation(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_grab')


        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0,0,0,0], wait=True)
        print("ready")
        rospy.sleep(2)

    def run(self):
        # move joint and gripper to grab the cylinder

        # TODO
        #arm_joint_goal = [0, math.radians(1.2), math.radians(-0.449), math.radians(-0.6)]
        #gripper_joint_open = [0.01, 0.01]
        arm_joint_goal = [0, 1.2, -0.449, 0.5]
        gripper_joint_open = [0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        gripper_joint_open = [-0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_open, wait=True)
        self.move_group_gripper.stop()


        arm_joint_goal = [0, 0.5, -0.3, 0.45]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()





if __name__ == "__main__":
    node = Manipulation()
    node.run()
