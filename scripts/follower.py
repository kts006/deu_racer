#!/usr/bin/env python
#-*- coding: utf-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
from geometry_msgs.msg import Twist
from deu_racer.msg import Deu_pose
from std_msgs.msg import Bool



class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.pose_msg = Deu_pose()
        self.pose_pub = rospy.Publisher('/line_pose',Deu_pose,queue_size = 1)
        self.stabilizer_pub = rospy.Publisher('/tb3/control/stabilize',Bool,queue_size = 1)
        self.image_sub = rospy.Subscriber('/python_image0/compressed', CompressedImage, self.image_callback)
        self.parking_sub = rospy.Subscriber('/tb3/control/parking', Bool, self.parking_callback)
        self.parkline_pub = rospy.Publisher('/tb3/control/parking_white',Bool,queue_size = 1)
        #self.image_sub = rospy.Subscriber('/python_image0',Image, self.image_callback)
        self.area = 7
        self.parking_topic_count = 0
        self.cnt = 0
        self.parking = False
        self.stabilizer = True
        if rospy.has_param('~area'):
            rospy.logdebug('%s', rospy.get_param('~area'))
            if rospy.get_param('~area') > 5:
                pass
            else:
                self.area = rospy.get_param('~area')

    def parking_callback(self, msg):
        self.parking = msg.data

    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='8UC3')
        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='8UC3')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow_line = np.array([ 24, 0, 220])
        upper_yellow_line = np.array([ 38, 150, 255])
        lower_white_line = np.array([ 47,  0,  207])
        upper_white_line = np.array([157, 47, 255])
        mask_left = cv2.inRange(hsv, lower_yellow_line, upper_yellow_line)
        mask_right = cv2.inRange(hsv, lower_white_line, upper_white_line)
        
        h, w, d = image.shape
        search_top = h/10*self.area
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
        
        if self.parking :
          indices = np.where(mask_right > 0)
          indices_len = len(indices[0])
          rospy.logdebug('pixel count : %d',indices_len)
          
          if indices_len < 1:
            print "======================================================================="
            self.parkline_pub.publish(True)
            rospy.logdebug('pub_True')
        
        
        if M_left['m00'] > 0 and M_right['m00'] <= 0 and not self.stabilizer:
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx = cx_left + 120
            cy = cy_left
            self.pose_msg.xpose = cx
            self.pose_msg.ypose = cy
            self.pose_pub.publish(self.pose_msg)
            image = cv2.circle(image, (cx_left, cy_left), 10, (0,0,255), -1)
            image = cv2.circle(image, (cx, cy), 10, (0,255,0), -1)
            
        elif M_left['m00'] <= 0 and M_right['m00'] > 0 and not self.stabilizer :
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cx = cx_right - 120
            cy = cy_right
            self.pose_msg.xpose = cx
            self.pose_msg.ypose = cy
            self.pose_pub.publish(self.pose_msg)
            image = cv2.circle(image, (cx_right, cy_right), 10, (255,0,0), -1)
            image = cv2.circle(image, (cx, cy), 10, (0,255,0), -1)
        
        elif M_left['m00'] > 0 and M_right['m00'] > 0:
            rospy.logdebug('both')
            self.stabilizer = False
            cx_left = int(M_left['m10']/M_left['m00'])
            cy_left = int(M_left['m01']/M_left['m00'])
            cx_right = int(M_right['m10']/M_right['m00'])
            cy_right = int(M_right['m01']/M_right['m00'])
            cx = (cx_left + cx_right)//2
            cy = (cy_left + cy_right)//2
            self.pose_msg.xpose = cx
            self.pose_msg.ypose = cy
            self.pose_pub.publish(self.pose_msg)
            image = cv2.circle(image, (cx_left, cy_left), 10, (0,0,255), -1)
            image = cv2.circle(image, (cx_right, cy_right), 10, (255,0,0), -1)
            image = cv2.circle(image, (cx, cy), 10, (0,255,0), -1)
            
        else:
            self.stabilizer = True
            self.stabilizer_pub.publish(self.stabilizer)
                
            
        #rospy.logdebug('X_pose:%d Y_pose:%d Stabilizer %s',self.pose_msg.xpose,self.pose_msg.ypose, self.stabilizer)       
        cv2.imshow("Navigation", image)
        cv2.imshow("mask_L", mask_left)
        cv2.imshow("mask_R", mask_right)
        cv2.waitKey(3)
        
rospy.init_node('follower',log_level=rospy.INFO)
follower = Follower()
rospy.spin()
# END ALL

