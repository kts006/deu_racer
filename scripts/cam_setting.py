#!/usr/bin/env python
#-*- coding: utf-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Bool

## 라인카메라 세팅용, 양쪽 라인이 중점으로 떨어진거리가 같을경우 메시지로 알려줌

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/python_image0/compressed', CompressedImage, self.image_callback)


    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='8UC3')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_line = np.array([ 0,  0,  160])
        upper_line = np.array([255, 255, 255])
        mask_left = cv2.inRange(hsv, lower_line, upper_line)
        mask_right = cv2.inRange(hsv, lower_line, upper_line)
        
        h, w, d = image.shape
        search_top = h/2
        search_bot = search_top + 20
        search_across = w/2
        search_left = w/3
        search_right = w*2/3

        mask_right[0:search_top, 0:w]=0
        mask_left[0:search_top, 0:w] = 0
        mask_right[search_bot:h, 0:w] = 0
        mask_left[search_bot:h, 0:w] = 0

        mask_left[0:h, search_across:w] = 0
        mask_right[0:h, 0:search_across] = 0
        M_left = cv2.moments(mask_left)
        M_right = cv2.moments(mask_right)
        
        if M_left['m00'] > 0 and M_right['m00'] > 0:
            self.stabilizer = False
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            image = cv2.circle(image, (cx_left, cy_left), 5, (0,0,255), -1)
            image = cv2.circle(image, (cx_right, cy_right), 5, (255,0,0), -1)
            
            rospy.logdebug('left %d right %d', w/2-cx_left, cx_right-w/2)
            
            if w/2-cx_left == cx_right-w/2 :
                rospy.logdebug('Camera Setting!!')
                   
        cv2.imshow("image", image)
        cv2.waitKey(3)
        
rospy.init_node('camera_setting',log_level=rospy.DEBUG)
follower = Follower()
rospy.spin()
# END ALL
