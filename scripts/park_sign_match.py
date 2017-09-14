#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from distance_calculator import DistanceCalculator
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool
import pyturtlebot as Robot


MIN_MATCH_COUNT = 10
show_matched_points = True

class Parking:
    def __init__(self):
    	self.count = 0
        self.cnt = 0
        self.had_been_park = 0
        self.bridge = cv_bridge.CvBridge()
        self.dist_calc = DistanceCalculator('parking')
        self.parking_state = False
        self.white_state = False

        self.surf = cv2.xfeatures2d.SURF_create(1000) # 물체 인식하려면 300~500이 적당
        self.blocking_img = cv2.imread('parking_marker.png', cv2.IMREAD_GRAYSCALE)
        self.image_sub = rospy.Subscriber('/python_image1/compressed', CompressedImage,  self.image_callback)
        self.white_sub = rospy.Subscriber('tb3/control/parking_white',Bool, self.white_callback)
        self.parking_pub = rospy.Publisher('tb3/control/parking',Bool,queue_size=1)
        self.kp1, self.des1 = self.surf.detectAndCompute(self.blocking_img, None)
        
    #parking_sub = True : activate
    def parking_move(self):
        robot = Robot.Move_controller()
        robot.parking_moving()

    def white_callback(self,msg):
        self.white_state = msg.data
        print "self.white_state", self.white_state,"self.parking_state",self.parking_state 
        if self.parking_state and self.white_state :
          if self.had_been_park < 1 : 
            self.parking_move()
            self.parking_state = False
            self.parking_pub.publish(self.parking_state)
            self.cnt = 0
            self.had_been_park = 1
       

    # 이미지 처리 후 표지판을 찾으면 /tb3/control/parking를 Publish
    def image_callback(self, msg):
	    self.count = self.count + 1
	    image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='8UC3')
	    imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	           
	    if self.count % 3 == 1:
		    cv2.useOptimized()
		    cv2.setUseOptimized(True)

		    kp2, des2 = self.surf.detectAndCompute(imageGray, None)

		    FLANN_INDEX_KDTREE = 1
		    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
		    search_params = dict(checks=50)
		    matches = None

		    try:
		        flann = cv2.FlannBasedMatcher(index_params, search_params)
		        matches = flann.knnMatch(self.des1, des2, k=2)

		    except Exception as ex:
		        print('knnMatch error')
		        return 
		        
		    good = []
		    for m, n in matches:
		        if m.distance < 0.75 * n.distance:
		            good.append(m)
		    outer_dst_pts = np.float32([])

		    if len(good) > MIN_MATCH_COUNT:
		        src_pts = np.float32([self.kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
		        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

		        outer_dst_pts = dst_pts
		        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
		        matchesMask = mask.ravel().tolist()
		        h, w= self.blocking_img.shape
		        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
		        dst = None
		        try:
		            dst = cv2.perspectiveTransform(pts, M)
		        except Exception as ex:
		            print('perspectiveTransform error: dst = %s' % dst)
		            return
		        if dst is None: return
		        image = cv2.polylines(image, [np.int32(dst)], True, (255, 0, 0), 3, cv2.LINE_AA)
	        
		        self.cnt = self.cnt + 1
		        if self.cnt > 10 and not self.parking_state:
		            self.parking_state = True
	                self.parking_pub.publish(self.parking_state)
		        rospy.logdebug('주차 표지판 탐지 : %s ' % self.parking_state)

		    else:
		        #self.parking_state = False
		        rospy.logdebug("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
		        matchesMask = None
		        
		    draw_params = dict(matchColor=(0, 255, 0),singlePointColor=None,matchesMask=matchesMask,flags=2)
		    matches_img = cv2.drawMatches(self.blocking_img, self.kp1, image, kp2, good, None, **draw_params)
		
		    if show_matched_points:
		        for pt in outer_dst_pts:
		            x,y = pt[0]
		            cv2.circle(image, (x, y),3, (0,0,255), -1)
            if self.cnt > 10 :
                self.parking_state = True
            
            

            cv2.imshow("park_sign_image", image)
            cv2.waitKey(1)
        

if __name__ == '__main__':
    rospy.init_node('sign',log_level=rospy.DEBUG)
    p = Parking()
    rospy.spin()

