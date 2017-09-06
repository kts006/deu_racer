#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2, cv_bridge
import match

class CameraSub:
    def __init__(self):
        self.i = 0
        self.match = match.Match()
        self.bridge =cv_bridge.CvBridge()
        #cv2.namedWindow("window",1)
        #image_sub = rospy.Subscriber('arm_sensor/camera/image_raw', Image, self.image_callback)
        image_sub = rospy.Subscriber('front_center_camera/image_raw/compressed', CompressedImage, self.image_callback)

    def image_callback(self,msg):
        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        res = self.match.matching(image)
        cv2.imshow("window",res)
        cv2.waitKey(3)
        
if __name__ == "__main__":
    rospy.init_node('camerasub')
    camerasub = CameraSub()
    rospy.spin()
  
  
