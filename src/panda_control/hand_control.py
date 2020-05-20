#!/usr/bin/env python

import sys
import rospy
import math
import tf
import moveit_commander
from geometry_msgs.msg import WrenchStamped

class ForceSensor(object):
    def __init__(self, topic):
        self.topic = topic
        self.listener = tf.TransformListener()

    def sum_of_forces(self):
        message = rospy.wait_for_message(self.topic, WrenchStamped)
        x = abs(message.wrench.force.x)
        y = abs(message.wrench.force.y)
        z = abs(message.wrench.force.z)
        return x+y+z


class HandControl(object):
    def __init__(self):
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group_hand = moveit_commander.MoveGroupCommander("hand")
        self.open_gripper()

        self.right_sensor = ForceSensor("/ft_right_sensor")
        self.left_sensor = ForceSensor("/ft_left_sensor")

    def open_gripper(self):
        joint_goal = self.move_group_hand.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        self.move_group_hand.go(joint_goal, wait=True)
        self.move_group_hand.stop()

    def close_gripper(self, dx):
        joint_goal = self.move_group_hand.get_current_joint_values()
        joint_goal[0] -= dx
        joint_goal[1] -= dx
        self.move_group_hand.go(joint_goal, wait=True)
        self.move_group_hand.stop()

    def update_arm_pose(self, dx):
        pose_goal = self.move_group_arm.get_current_pose().pose
        angles = tf.transformations.euler_from_quaternion([pose_goal.orientation.x,
        pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w])
        theta = angles[2]

        direction = 0
        if self.right_sensor.sum_of_forces() > self.left_sensor.sum_of_forces():
            direction = -1
        else:
            direction = 1

        pose_goal.position.x += direction*dx*math.sin(theta)
        pose_goal.position.y += direction*dx*math.cos(theta)

        self.move_group_arm.set_pose_target(pose_goal)

        plan = self.move_group_arm.plan()
        if plan.joint_trajectory.points:
            move_success = self.move_group_arm.execute(plan)
        else:
            rospy.logerr("Trajectory is empty. Planning was unsuccessful.")

        self.move_group_arm.stop()
        self.move_group_arm.clear_pose_targets()

    def impendance_control(self):
        while (self.left_sensor.sum_of_forces()+self.right_sensor.sum_of_forces()) < 0.01:
            self.close_gripper(0.001);

        while(abs(self.left_sensor.sum_of_forces()-self.right_sensor.sum_of_forces())) > 0.005:
            rospy.loginfo("Moving the arm")
            self.update_arm_pose(0.005)
            rospy.loginfo(self.left_sensor.sum_of_forces())
            rospy.loginfo(self.right_sensor.sum_of_forces())
