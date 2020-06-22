#!/usr/bin/env python

import sys
import rospy
import ros_numpy
import math
import tf
import moveit_commander
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from controller_manager_msgs.srv import SwitchController
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros

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
        self.gripper__width_pub = rospy.Publisher("/gripper_width", Float64, queue_size=5)
        self.left_pub = rospy.Publisher("/left_force_module", Float64, queue_size=10)
        while not self.gripper_pub.get_num_connections():
            rospy.sleep(1)

        self.gripper_current = [0,0]
        self.open_gripper()

        self.right_sensor = ForceSensor("/ft_right_sensor")
        self.left_sensor = ForceSensor("/ft_left_sensor")

        self.force_ref_x = 1
        self.force_ref_y = 1
        self.force_ref_z = 0.1
        self.force_ref_module = 1.4

        self.new_pc_points = np.empty((0,3))

    def switch_controller(self, start_controllers, stop_controllers):
		rospy.wait_for_service('/controller_manager/switch_controller')
		try:
			switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
			ret = switch_controller(start_controllers, stop_controllers, 2, True, 5)
			rospy.loginfo("Controllers switched")
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

    def plot_point_cloud(self):
        points = ros_numpy.point_cloud2.get_xyz_points(self.cloud_array)
        points = points[points[:,0] > 0.3]
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(points[:,2], points[:,1], points[:,0], c='r', marker='o')
        ax.scatter(self.new_pc_points[:,0], self.new_pc_points[:,1], self.new_pc_points[:,2], s=100, c='k', marker='o')
        ax.set_xlabel('x-os')
        ax.set_ylabel('y-os')
        ax.set_zlabel('z-os')
        plt.show()

    def read_point_cloud(self):
        self.pc = rospy.wait_for_message('/camera/depth/points', PointCloud2)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        while not rospy.is_shutdown():
            try:
                transform = tf_buffer.lookup_transform('panda_camera_link','world', rospy.Time())
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        self.pc_world = do_transform_cloud(self.pc, transform)
        self.cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_world)

    def publish_point_cloud(self):
        pcl = ros_numpy.point_cloud2.array_to_pointcloud2(self.cloud_array, stamp=None, frame_id='world')
        self.pcl_pub = rospy.Publisher("/updated_point_cloud", PointCloud2, queue_size=5)
        self.pcl_pub.publish(pcl)

    def update_point_cloud(self):
        x = -1*np.float32(self.left_finger_pose.pose.position.x)
        y = -1*np.float32(self.left_finger_pose.pose.position.y)
        z = np.float32(self.left_finger_pose.pose.position.z)-0.06
        rgb = np.float32(0)

        self.new_pc_points = np.append(self.new_pc_points, np.array([[x,y,z]]), axis=0)
        #self.cloud_array = np.append(self.cloud_array, np.array([x,y,z,rgb], dtype=self.cloud_array.dtype))

        x = -1*np.float32(self.right_finger_pose.pose.position.x)
        y = -1*np.float32(self.right_finger_pose.pose.position.y)
        z = np.float32(self.right_finger_pose.pose.position.z)-0.06
        rgb = np.float32(0)

        self.new_pc_points = np.append(self.new_pc_points, np.array([[x,y,z]]), axis=0)
        rospy.logerr(self.new_pc_points.shape)
        #self.cloud_array = np.append(self.cloud_array, np.array([x,y,z,rgb], dtype=self.cloud_array.dtype))

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
        self.gripper__width_pub.publish(0.08)

    def update_width (self, dx):
        rospy.loginfo("Changing gripper width for: %f", dx)

        if (self.gripper_current[0] > 0.039 or self.gripper_current[1] > 0.039) and dx > 0:
            self.open_gripper()
            rospy.loginfo("Gripper width reached maximum!")
            rospy.sleep(1)
            return 0

        self.gripper_current[0] += dx
        self.gripper_current[1] += dx
        array = Float64MultiArray(data=self.gripper_current)
        self.gripper_pub.publish(array)
        self.gripper__width_pub.publish(self.gripper_current[1]*2)
        rospy.sleep(1)
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
        left_module = self.left_sensor.sum_of_forces()
        self.left_pub.publish(left_module)
        if abs(left_module-self.force_ref_module) < 0.1:
            self.starter(False)
            return

        direction = self.pose_determine_direction(data, self.left_finger_pose, self.right_finger_pose)
        if not self.update_width(direction*0.0001):
            self.starter(False)
            rospy.sleep(3)
        else:
            self.publish_pose()

    def equal_to_current_pose(self, data):
        xc = self.left_sensor.pose.position.x
        yc = self.left_finger_pose.pose.position.y
        zc = self.left_finger_pose.pose.position.z

        xr = data.position.x
        yr = data.position.y
        zr = data.position.z

        if math.sqrt((xc-xr)**2 + (yc-yr)**2 + (zc-zr)**2) < 0.001:
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
        while (self.left_sensor.sum_of_forces()+self.right_sensor.sum_of_forces()) < 1:
            self.left_pub.publish(self.left_sensor.sum_of_forces())
            self.update_width(-0.001);

        while(abs(self.left_sensor.sum_of_forces()-self.right_sensor.sum_of_forces())) > 0.5:
            self.left_pub.publish(self.left_sensor.sum_of_forces())
            rospy.loginfo("Moving the arm")
            direction = self.force_determine_direction()
            self.update_arm_pose(0.005, direction)
            rospy.loginfo(self.left_sensor.sum_of_forces())
            rospy.loginfo(self.right_sensor.sum_of_forces())

        while not self.pub_pose.get_num_connections():
            rospy.sleep(1)

        self.left_pub.publish(self.left_sensor.sum_of_forces())
        self.publish_pose()
        self.update_point_cloud()
        self.plot_point_cloud()
        self.publish_point_cloud()

        rospy.wait_for_service('/impedance_control/start')
        self.starter = rospy.ServiceProxy('/impedance_control/start', SetBool)
        self.starter(True)

        rospy.Subscriber('/impedance_control/pose_output', Pose, self.pose_callback, queue_size=1)
        rospy.spin()
