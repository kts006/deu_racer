#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy, cv2, cv_bridge,numpy
import numpy as np
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
import logging
from elapsed_time import ElapsedTime
from distance_calculator import DistanceCalculator
from rgb_color import RGBColor


MIN_MATCH_COUNT = 10

class parking_publisher:
  def __init__(self):

    self.bridge = cv_bridge.CvBridge()

    self.logger = logging.getLogger('cv_test')
    logging.basicConfig(level=logging.INFO)

    self.dist_calc = DistanceCalculator('parking')
    self.show_matcher_points = True
    self.timer = ElapsedTime()

    self.show_matched_points = True

    self.sift = cv2.xfeatures2d.SIFT_create()
    self.img1 = cv2.imread('parking_marker.png', cv2.IMREAD_COLOR)
    if self.img1 is None:
      print 'img1 is deleted'
    self.kp1, self.des1 = self.sift.detectAndCompute(self.img1, None)

    self.image_sub = rospy.Subscriber('/center_camera/image_raw/compressed', CompressedImage, self.find_callback)
    self.key_pub = rospy.Publisher('t3burger/control/parking', Bool, queue_size=1)
    rospy.init_node("test_driver")
    self.rate = rospy.Rate(100)

  def create_parking_publish(self):
    #real function part
    self.key_pub.publish(True)
    self.rate.sleep()

  def find_callback(self,msg):
    start_t = self.timer.start()
    frame = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')

    img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    kp2, des2 = self.sift.detectAndCompute(img2, None)
    
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(self.des1, des2, k=2)
    # store all the good matches as per Lowe's ratio test.
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    outer_dst_pts = np.float32([])
    if len(good) > MIN_MATCH_COUNT:
        pubs.create_parking_publish()

    end_t = self.timer.end()

    
if __name__ == '__main__':
  pubs = parking_publisher()
  while not rospy.is_shutdown():
    rospy.spin()

