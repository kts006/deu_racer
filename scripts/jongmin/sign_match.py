# -*- coding: utf-8 -*-
#! /usr/bin/env python

import rospy
import cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool


MIN_MATCH_COUNT = 10
show_matched_points = True


class Sign:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        
        # 물체 인식하려면 300~500이 적당
        self.sift = cv2.xfeatures2d.SIFT_create()

        self.blocking_img = cv2.imread('parking_marker.png', cv2.IMREAD_COLOR)

        #self.image_sub = rospy.Subscriber('center_camera/image_raw/compressed', CompressedImage, self.image_callback)
        self.match_pub = rospy.Publisher('tb3/control/parking',Bool)
        
        self.image_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage,  self.image_callback)
        
        self.match = False


    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        cv2.useOptimized()
        cv2.setUseOptimized(True)

        kp1, des1 = self.sift.detectAndCompute(self.blocking_img, None)
        kp2, des2 = self.sift.detectAndCompute(imageGray, None)
        
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)

        try:
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des1, des2, k=2)

        except Exception as ex:
            print('knnMatch error')
            
            return 
            
        good = []

        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        outer_dst_pts = np.float32([])

        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            outer_dst_pts = dst_pts

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            matchesMask = mask.ravel().tolist()

            h, w, d= self.blocking_img.shape

            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

#            dst = None

            try:
                dst = cv2.perspectiveTransform(pts, M)

            except Exception as ex:
                print('perspectiveTransform error: dst = %s' % dst)
                
                return
             
            image = cv2.polylines(image, [np.int32(dst)], True, (255, 0, 0), 3, cv2.LINE_AA)
            
            self.match = True
            
            rospy.logdebug('주차 표지판 탐지 : %s' % self.match)
            
        else:
            self.match = False
            
            rospy.logdebug("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))

            matchesMask = None

        #draw_params = dict(matchColor=(0, 255, 0),
        #                   singlePointColor=None,
        #                   matchesMask=matchesMask,
        #                   flags=2)

       # matches_img = cv2.drawMatches(self.blocking_img, kp1, image, kp2, good, None, **draw_params)
        
        if show_matched_points:
            for pt in outer_dst_pts:
                x,y = pt[0]

                cv2.circle(image, (x, y),3, (0,0,255), -1)
        
        self.match_pub.publish(self.match)
        
        #cv2.imshow('match', matches_img)
        cv2.imshow("image", image)

        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('sign',log_level=rospy.DEBUG)
    
    sign = Sign()

    rospy.spin()
