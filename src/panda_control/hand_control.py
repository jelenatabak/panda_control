#!/usr/bin/env python

import sys
import rospy
import math
import tf
import moveit_commander
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from controller_manager_msgs.srv import SwitchController

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
        self.move_group_arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.listener = tf.TransformListener()

        self.position_controller = ['gripper_position_controller']
        self.trajectory_controller = ['panda_hand_controller']
        self.switch_controller(self.position_controller, self.trajectory_controller)

        self.gripper_pub = rospy.Publisher("/gripper_position_controller/command", Float64MultiArray, queue_size=5)
        while not self.gripper_pub.get_num_connections():
            rospy.sleep(1)

        self.gripper_current = [0,0]
        self.open_gripper()

        self.right_sensor = ForceSensor("/ft_right_sensor")
        self.left_sensor = ForceSensor("/ft_left_sensor")

    def switch_controller(self, start_controllers, stop_controllers):
		rospy.wait_for_service('/controller_manager/switch_controller')
		try:
			switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
			ret = switch_controller(start_controllers, stop_controllers, 2, True, 5)
			rospy.loginfo("Controllers switched")
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

    def publish_pose(self):
        while not rospy.is_shutdown():
            try:
                position,quaternion = self.listener.lookupTransform('/world', '/panda_leftfinger', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Waiting for the transformation to become available..")

        self.left_finger_pose = PoseStamped()
        self.left_finger_pose.header.frame_id = "panda_leftfinger"
        self.left_finger_pose.header.stamp = rospy.Time.now()
        self.left_finger_pose.pose = Pose(position=Point(x=position[0],y=position[1],z=position[2]),
                            orientation=Quaternion(x=quaternion[0],y=quaternion[1],z=quaternion[2],w=quaternion[3]))
        self.pub_pose.publish(self.left_finger_pose)

        while not rospy.is_shutdown():
            try:
                position,quaternion = self.listener.lookupTransform('/world', '/panda_rightfinger', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Waiting for the transformation to become available..")

        self.right_finger_pose = PoseStamped()
        self.right_finger_pose.header.frame_id = "panda_rightfinger"
        self.right_finger_pose.header.stamp = rospy.Time.now()
        self.right_finger_pose.pose = Pose(position=Point(x=position[0],y=position[1],z=position[2]),
                            orientation=Quaternion(x=quaternion[0],y=quaternion[1],z=quaternion[2],w=quaternion[3]))
        #self.pub_pose.publish(self.left_finger_pose)


    def open_gripper(self):
        array = Float64MultiArray(data=[0.04, 0.04])
        self.gripper_current = [0.04,0.04]
        self.gripper_pub.publish(array)

    def update_width (self, dx):
        rospy.loginfo("Changing gripper width for: %f", dx)

        if (self.gripper_current[0] > 0.039 or self.gripper_current[1] > 0.039) and dx > 0:
            self.open_gripper()
            rospy.loginfo("Gripper width reached maximum!")
            return 0

        self.gripper_current[0] += dx
        self.gripper_current[1] += dx
        array = Float64MultiArray(data=self.gripper_current)
        self.gripper_pub.publish(array)
        return 1

    def update_arm_pose(self, dx, direction):
        pose_goal = self.move_group_arm.get_current_pose().pose
        angles = tf.transformations.euler_from_quaternion([pose_goal.orientation.x,
        pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w])
        theta = angles[2]

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

    def force_determine_direction(self):
        if self.right_sensor.sum_of_forces() > self.left_sensor.sum_of_forces():
            return -1
        else:
            return 1

    def pose_callback(self, data):
        if self.equal_to_current_pose(data):
            self.starter(False)
            return

        direction = self.pose_determine_direction(data, self.left_finger_pose, self.right_finger_pose)
        if not self.update_width(direction*0.001):
            self.starter(False)
        else:
            self.publish_pose()
            rospy.sleep(3)

    def equal_to_current_pose(self, data):
        xc = self.left_finger_pose.pose.position.x
        yc = self.left_finger_pose.pose.position.y
        zc = self.left_finger_pose.pose.position.z

        xr = data.position.x
        yr = data.position.y
        zr = data.position.z

        if math.sqrt((xc-xr)**2 + (yc-yr)**2 + (zc-zr)**2) < 0.0001:
            return True
        else:
            return False

    def pose_determine_direction(self, ref_pose, finger_pose, other_finger_pose):
        xr = ref_pose.position.x
        yr = ref_pose.position.y
        xc = finger_pose.pose.position.x
        yc = finger_pose.pose.position.y
        xo = other_finger_pose.pose.position.x
        yo = other_finger_pose.pose.position.y

        ref_width = math.sqrt((xo-xr)**2 + (yo-yr)**2)
        current_width =  math.sqrt((xo-xc)**2 + (yo-yc)**2)

        if ref_width < current_width:
            return -1
        else:
            return 1

    def impendance_control(self):
        while (self.left_sensor.sum_of_forces()+self.right_sensor.sum_of_forces()) < 0.1:
            self.update_width(-0.001);

        while(abs(self.left_sensor.sum_of_forces()-self.right_sensor.sum_of_forces())) > 0.05:
            rospy.loginfo("Moving the arm")
            direction = self.force_determine_direction()
            self.update_arm_pose(0.005, direction)
            rospy.loginfo(self.left_sensor.sum_of_forces())
            rospy.loginfo(self.right_sensor.sum_of_forces())

        while not self.pub_pose.get_num_connections():
            rospy.sleep(1)

        self.publish_pose()

        rospy.wait_for_service('/impedance_control/start')
        self.starter = rospy.ServiceProxy('/impedance_control/start', SetBool)
        self.starter(True)

        rospy.Subscriber('/impedance_control/pose_output', Pose, self.pose_callback)
        rospy.spin()

        # pomaknut ruku nakon zatvaranja hvataljke tako da na desnom senzoru nema promjena
        # kreirat impedance_control i za desni senzor (iz launcha?)
        # za desni raditi pomake ruke, a hvataljku podesavati tako da nema promjene na lijevom
