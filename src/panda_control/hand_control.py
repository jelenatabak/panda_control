#!/usr/bin/env python

import sys
import rospy
import math
import tf
import moveit_commander
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class ForceSensor(object):
    def __init__(self, topic):
        self.topic = topic
        self.listener = tf.TransformListener()

    def sum_of_forces(self):
        message = rospy.wait_for_message(self.topic, WrenchStamped)
        x = message.wrench.force.x
        y = message.wrench.force.y
        z = message.wrench.force.z
        return math.sqrt(x*x+y*y+z*z)


class HandControl(object):
    def __init__(self):
        self.pub_pose = rospy.Publisher("/impedance_control/pose_stamped_ref_input", PoseStamped, queue_size=5)
        self.listener = tf.TransformListener()

        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.move_group_hand = moveit_commander.MoveGroupCommander("hand")
        self.open_gripper()

        self.right_sensor = ForceSensor("/ft_right_sensor")
        self.left_sensor = ForceSensor("/ft_left_sensor")

    def publish_pose(self):
        while not rospy.is_shutdown():
            try:
                position,quaternion = self.listener.lookupTransform('/world', '/panda_leftfinger', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Waiting for the transformation to become available..")

        pose = PoseStamped()
        pose.header.frame_id = "panda_leftfinger"
        pose.header.stamp = rospy.Time.now()
        pose.pose = Pose(position=Point(x=position[0],y=position[1],z=position[2]),
                            orientation=Quaternion(x=quaternion[0],y=quaternion[1],z=quaternion[2],w=quaternion[3]))
        self.pub_pose.publish(pose)

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

        while(abs(self.left_sensor.sum_of_forces()-self.right_sensor.sum_of_forces())) > 0.05:
            rospy.loginfo("Moving the arm")
            self.update_arm_pose(0.005)
            rospy.loginfo(self.left_sensor.sum_of_forces())
            rospy.loginfo(self.right_sensor.sum_of_forces())

        while not self.pub_pose.get_num_connections():
            rospy.logerr("Waiting for connections, run impedance_control!")
            rospy.sleep(1)

        self.publish_pose()

        rospy.wait_for_service('/impedance_control/start')
        starter = rospy.ServiceProxy('/impedance_control/start', SetBool)
        starter(True)
