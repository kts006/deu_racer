#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2, cv_bridge, numpy

from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage, Image


class Camera:
    def __init__(self):    
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('center_camera/image_raw/compressed', CompressedImage, self.image_callback)
        self.image_sub = rospy.Subscriber('/python_image1/compressed', CompressedImage, self.image_callback)
        self.blocking_pub = rospy.Publisher('tb3/control/block',Bool, queue_size=1)
        
        #self.image_sub = rospy.Subscriber('camera/image_raw/compressed', CompressedImage,  self.image_callback)
        
        self.blocking = False

    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = numpy.array([0, 115, 115])
        upper_red = numpy.array([3, 255, 255])

        mask0 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = numpy.array([170, 115, 115])
        upper_red = numpy.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask0 + mask1

        #라벨링
        kernel = numpy.ones((15, 15), numpy.uint8)
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        labelnum, labelimg, contours, GoCs = cv2.connectedComponentsWithStats(closing)

        max_indices_len = 0
        max_label = 0

        for label in xrange(1, labelnum):
            label_indices = numpy.where(labelimg == label)
            label_indices_len = len(label_indices[0])

            if label_indices_len > max_indices_len:
                max_indices_len = label_indices_len
                max_label = label

        circle_x, circle_y = GoCs[max_label]
        x, y, w, h, size = contours[max_label]

        image = cv2.circle(image, (int(circle_x), int(circle_y)), 5, (0, 255, 0), -1)
        image = cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)

        # 픽셀 수 (20cm: 10200~10400, 25cm: 6300~6500, 30cm: 4200~4500, 35cm: 2900~3100, 40cm: 2300~2500) 
        print 'w*h= ', w*h, 'size= ', size, 'h= ',h


        if (w*h) - 500 <= size and size < 6000 and 25 < h:

            if 2000 < max_indices_len:
                self.blocking = True
                
                rospy.loginfo('차단바 탐지')     
       
        else:
            self.blocking = False

        self.blocking_pub.publish(self.blocking)

        rospy.logdebug('최대 픽셀 수 = %d, msg %s' % (max_indices_len, self.blocking))
       
        cv2.imshow("blocking", image)
        cv2.imshow('closing', closing)
        
        cv2.waitKey(3)


if __name__ == "__main__":
    rospy.init_node('camera',log_level=rospy.DEBUG)
    
    camera = Camera()

    rospy.spin()

