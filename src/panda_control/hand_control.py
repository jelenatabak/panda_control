#!/usr/bin/env python

import sys
import rospy
import math
import tf
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point, Quaternion

class HandControl(object):
    def __init__(self):
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group_hand = moveit_commander.MoveGroupCommander("hand")
        self.open_gripper()

    def open_gripper(self):
        joint_goal = self.move_group_hand.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        self.move_group_hand.go(joint_goal, wait=True)
        self.move_group_hand.stop()

	def go_to_pose_goal(self):
		self.move_group_arm.set_pose_target(self.pose_goal)

		plan = self.move_group_arm.plan()
		if plan.joint_trajectory.points:
				move_success = self.move_group_arm.execute(plan)
		else:
		  rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

		self.move_group_arm.stop()
		self.move_group_arm.clear_pose_targets()

    def impendance_controller(self):
        # ft_sensor = rospy.wait_for_message("", ...)
        # dok su oba senzora u nuli, zatvaraj hvataljku
        # ako je jedan u 0 a drugi nije, pomakni po osi hvataljke - postavi self.pose_goal i pozovi go_to_pose_goal()
        # impedantno
        return
