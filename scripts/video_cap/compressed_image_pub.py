#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np

def image_publisher(device_id=0, frame_rate=20, width=320, height=240, brightness=0.5):
   topic_name = "python_image" + str(device_id)
   node_name = 'python_image_publisher' + str(device_id)
   pub1 = rospy.Publisher(topic_name, Image, queue_size=10)
   pub2 = rospy.Publisher(topic_name + "/compressed", CompressedImage, queue_size=10)
   rospy.init_node(node_name)
   rate = rospy.Rate(frame_rate)
   
   cap = cv2.VideoCapture(device_id)
   cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
   cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
   cap.set(cv2.CAP_PROP_FPS, frame_rate)
   cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
   # cap.set(cv2.CAP_PROP_WHITE_BALANCE, False)
   br = CvBridge()
   while not rospy.is_shutdown():
      status, img = cap.read()
      if status == True:
         pub1.publish(br.cv2_to_imgmsg(img)) 
         msg = CompressedImage()
         msg.header.stamp = rospy.Time.now()
         msg.format = "jpeg"  # "jpeg" or "png"
         msg.data = np.array(cv2.imencode(".jpg", img)[1]).tostring()
         pub2.publish(msg)
      rate.sleep()

if __name__ == "__main__":
   my_device_id = int(rospy.get_param("/image_pub/device_id", "0"))
   my_width = int(rospy.get_param("/image_pub/image_width", "640"))
   my_height = int(rospy.get_param("/image_pub/image_height", "480"))
   my_bright = float(rospy.get_param("/image_pub/bright"))
   
   print "image_publisher using opencv started..."
   print "device id =", my_device_id
   print "width =", my_width, "height =", my_height
   print "my_bright =", my_bright
   
   try:
      image_publisher(my_device_id, width=my_width, height=my_height,  brightness=my_bright)
   except rospy.ROSInterruptException:
      pass

