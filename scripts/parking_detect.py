#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy, cv2, cv_bridge,numpy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.image_sub = rospy.Subscriber('front_center_camera/image_raw', Image, self.image_callback)
    self.image_sub2 = rospy.Subscriber('front_left_camera/image_raw', Image, self.image_slice)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)#find center
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)#draw at center
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    cv2.imshow("window", image)
    cv2.waitKey(3)

  def ipm(self,img):
    pts1 = np.float32([[400,300],[500,300],[200,520],[830,520]])
    pts2 = np.float32([[0,0],[300,0],[0,300],[300,300]])
    m = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img, m, (300,300))

  def filter_region(self,image,vertices):
    """
    Create the mask using the vertices and apply it to the input image
    """
    mask = np.zeros_like(image)
    if len(mask.shape)==2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,)*mask.shape[2]) # in case, the input image has a channel dimension        
    return cv2.bitwise_and(image, mask)
  
  def lower_select_region(self,image,state):
    """
    It keeps the region surrounded by the `vertices` (i.e. polygon).  Other area is set to 0 (black).
    """
    if state == True:
      # first, define the polygon by vertices
      rows, cols = image.shape[:2]
      bottom_left  = [cols*0.1, rows*0.85]
      top_left     = [cols*0.4, rows*0.8]
      bottom_right = [cols*0.9, rows*0.85]
      top_right    = [cols*0.6, rows*0.8]
    else :
      rows, cols = image.shape[:2]
      bottom_left  = [cols*0.1, rows*0.7]
      top_left     = [cols*0.4, rows*0.68]
      bottom_right = [cols*0.9, rows*0.7]
      top_right    = [cols*0.6, rows*0.68]

    # the vertices are an array of polygons (i.e array of arrays) and the data type must be integer
    vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
    return self.filter_region(image, vertices)

  
  def image_slice(self,msg): 
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    edges = cv2.Canny(image,50,150)

    low_roi = self.lower_select_region(edges,True)
    up_roi = self.lower_select_region(edges, False)

    cv2.imshow("low_win",low_roi)
    cv2.imshow("up_win",up_roi)
    cv2.waitKey(3)

     

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

