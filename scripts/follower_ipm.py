#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, genpy
import numpy as np
from std_msgs.msg import String, Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from tf.msg import tfMessage


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.twist = Twist()
    self.tf_twist = Twist()
    self.image_sub = rospy.Subscriber('/front_camera/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    rows,cols,ch = image.shape
    pts1 = np.float32([[198,0],[460,0],[0,190],[640,170]])
    pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(image,M,(300,300))
    
    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 0,  0,  240])
    upper_yellow = np.array([255, 255, 255])
    mask_left = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask_right = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = dst.shape
    search_top   = h/3
    search_bot   = h/3 + 20
    search_across  = w/2 
    mask_right[0:search_top, 0:w]=0
    mask_left[0:search_top, 0:w] = 0
    mask_right[search_bot:h, 0:w] = 0
    mask_left[search_bot:h, 0:w] = 0

    mask_left[0:h, search_across:w] = 0
    mask_right[0:h, 0:search_across] = 0
    M_left = cv2.moments(mask_left)
    M_right = cv2.moments(mask_right)
    if M_left['m00'] > 0 and M_right['m00'] > 0:
        cx_left = int(M_left['m10']/M_left['m00'])
        cy_left = int(M_left['m01']/M_left['m00'])
        cx_right = int(M_right['m10']/M_right['m00'])
        cy_right = int(M_right['m01']/M_right['m00'])
        cx = (cx_left + cx_right)//2
        cy = (cy_left + cy_right)//2
        image2 = cv2.circle(dst, (cx_left, cy_left), 20, (0,0,255), -1)
        image3 = cv2.circle(image2, (cx_right, cy_right), 20, (255,0,0), -1)
        image4 = cv2.circle(image3, (cx, cy), 20, (0,255,0), -1)
        
        
        # BEGIN CONTROL
        err = cx - w/2
        self.twist.linear.x = 0.05
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
    
    cv2.imshow("dst",dst)
    cv2.imshow("hsv", hsv)
    cv2.imshow("image", image)
    cv2.imshow("right", mask_right)
    cv2.imshow("left", mask_left)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
