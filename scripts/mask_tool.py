#-*- coding: utf-8 -*-
import rospy, cv2, cv_bridge
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import logging
import os

    #영상의 HSV 영상의 mask를 추출하기 위한툴
class Light:

    def __init__(self):
        self.image_sub = rospy.Subscriber('/python_image1/compressed', CompressedImage, self.image_callback)
        

    def nothing(self,x):
        pass
        
    def create_trackbar_window(self):
        cv2.namedWindow('Trackbar', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Low H', 'Trackbar', 24, 179, self.nothing)
        cv2.createTrackbar('High H', 'Trackbar', 38, 179,self.nothing)
        cv2.createTrackbar('Low S', 'Trackbar', 0, 255, self.nothing)
        cv2.createTrackbar('High S', 'Trackbar', 150, 255, self.nothing)
        cv2.createTrackbar('Low V', 'Trackbar', 220, 255, self.nothing)
        cv2.createTrackbar('High V', 'Trackbar', 255, 255, self.nothing)
        
    def get_tracker(self,hsv):
        low_h = cv2.getTrackbarPos('Low H', 'Trackbar')
        high_h = cv2.getTrackbarPos('High H', 'Trackbar')
        low_s = cv2.getTrackbarPos('Low S', 'Trackbar')
        high_s = cv2.getTrackbarPos('High S', 'Trackbar')
        low_v = cv2.getTrackbarPos('Low V', 'Trackbar')
        high_v = cv2.getTrackbarPos('High V', 'Trackbar')
        
        filter_lower = np.array([low_h, low_s, low_v])
        filter_upper = np.array([high_h, high_s, high_v])
        
        mask = cv2.inRange(hsv, filter_lower, filter_upper)
        return mask
        
    def image_callback(self, msg):
        self.create_trackbar_window()
        bridge = cv_bridge.CvBridge()
        image = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask = self.get_tracker(hsv)
        cv2.imshow('image', image)
        cv2.imshow('mask', mask)
        cv2.waitKey(3)
        

if __name__ == '__main__':
    
        
    rospy.init_node('hsv_tool',log_level=rospy.DEBUG)

    light = Light()
    rospy.spin()



