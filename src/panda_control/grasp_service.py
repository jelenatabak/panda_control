#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from panda_control.srv import *

import tensorflow as tf
from tensorflow.keras.models import load_model
from tensorflow.keras.applications.resnet50 import preprocess_input
import cv2
import numpy as np
import rospkg

class GraspService(object):
    def __init__(self):
        self.s = rospy.Service('get_grasp_rectangle', GraspSrv, self.get_grasp_rectangle)
        self.bridge = CvBridge()

        self.config = tf.ConfigProto(device_count={'GPU': 0})
        self.session = tf.Session(config=self.config)
        tf.keras.backend.set_session(self.session)

        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('panda_control')
        self.model_path = self.pkg_path + '/models/grasp_model.h5'

        self.model = load_model(self.model_path)
        self.model._make_predict_function()

    def draw_bbox(self, image, bbox):
        img = image.copy()
        cv2.line(img, (bbox[0],bbox[1]), (bbox[2],bbox[3]), (255, 0, 0))
        cv2.line(img, (bbox[2],bbox[3]), (bbox[4],bbox[5]), (0, 0, 255))
        cv2.line(img, (bbox[4],bbox[5]), (bbox[6],bbox[7]), (255, 0, 0))
        cv2.line(img, (bbox[6],bbox[7]), (bbox[0],bbox[1]), (0, 0, 255))
        return img

    def draw_grasp(self, image, grasp):
        centre = [grasp[0],grasp[1]]
        height, width, theta = grasp[2], grasp[3], grasp[4]
        c, s = np.cos(theta), np.sin(theta)
        R = np.matrix('{} {}; {} {}'.format(c, -s, s, c))
        p1 = [ + height / 2,  + width / 2]
        p2 = [- height / 2,  + width / 2]
        p3 = [ - height / 2, - width / 2]
        p4 = [ + height / 2,  - width / 2]
        p1_new = np.dot(p1, R)+ centre
        p2_new = np.dot(p2, R)+ centre
        p3_new = np.dot(p3, R)+ centre
        p4_new = np.dot(p4, R)+ centre

        bbox = [p1_new, p2_new, p3_new, p4_new]
        bbox = np.array(bbox)
        bbox = bbox.reshape((1,8))[0].astype(int)
        return self.draw_bbox(image,bbox)

    def crop_center(self, img, x_c, y_c, size):
        startx = int(x_c-(size/2))
        starty = int(y_c-(size/2))
        return img[starty:starty+size, startx:startx+size,:]

    def get_grasp_rectangle(self, req):
        image_message = rospy.wait_for_message("/camera/color/image_raw", Image)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)

        x_c = int(cv_image.shape[1]/2)
        y_c = int(cv_image.shape[0]/2)

        cv_image_crop = self.crop_center(cv_image, x_c, y_c, 224)
        cv_image_pp = preprocess_input(cv_image_crop)
        cv_image_pp = np.expand_dims(cv_image_pp, 0)

        with self.session.as_default():
            with self.session.graph.as_default():
                grasp = self.model.predict(cv_image_pp)

        grasp = grasp[0]
        rospy.loginfo(grasp)

        labeled_image = self.draw_grasp(cv_image_crop, grasp)
        #cv2.namedWindow('predicted_grasp', cv2.WINDOW_NORMAL)
        #cv2.imshow('predicted_grasp', cv2.cvtColor(labeled_image, cv2.COLOR_BGR2RGB))
        #cv2.waitKey(100)

        return list(grasp)

    def call(self):
        try:
            get_grasp_rectangle_serer = rospy.ServiceProxy('get_grasp_rectangle', GraspSrv)
            grasp = get_grasp_rectangle_serer()
            return grasp

    	except rospy.ServiceException, e:
            print "Service call failed: %s"%e
