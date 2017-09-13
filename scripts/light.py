#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

class Light:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/python_image1/compressed', CompressedImage, self.image_callback)
        self.light_pub = rospy.Publisher('tb3/control/block', Bool)
        self.light = False

    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #image = cv2.GaussianBlur(image, (7,7),0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #hsv = cv2.GaussianBlur(hsv, (7, 7), 0)
        #hsv = cv2.medianBlur(hsv, 5)
        ######## red 

        lower_red = numpy.array([  0, 120, 220])
        upper_red = numpy.array([ 10, 160, 255])

        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        ########

        ######## yellow

        lower_yellow = numpy.array([  8,  57, 216])
        upper_yellow = numpy.array([ 35,  97, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        ########    

        ######## green 

        lower_green = numpy.array([70, 115, 225])
        upper_green = numpy.array([100,200, 255])

        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        ########

        Rmasked = cv2.bitwise_and(image, image, mask=mask_red)
        Ymasked = cv2.bitwise_and(image, image, mask=mask_yellow)
        Gmasked	= cv2.bitwise_and(image, image, mask=mask_green)

        RM = cv2.moments(mask_red)
        YM = cv2.moments(mask_yellow)
        GM = cv2.moments(mask_green)

        kernel = numpy.ones((20, 20), numpy.uint8)
        closing = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

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

        cv2.circle(image, (int(circle_x), int(circle_y)), 2/x, (0, 255, 0), 3)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)

        if ((2/x)*(2/x)*3.14) <= size and size <76800:
            if 1000 <max_indices_len:
                if RM['m00'] > 0:
                    self.Light = True
                    rospy.logdebug('red light on')
                    cx = int(RM['m10'] / RM['m00'])
                    cy = int(RM['m01'] / RM['m00'])
                    cv2.circle(image, (cx, cy), 20, (255, 0, 0), -1)

        kernel = numpy.ones((20, 20), numpy.uint8)
        closing = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

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

        cv2.circle(image, (int(circle_x), int(circle_y)), 2 / x, (0, 255, 0), 3)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)

        if ((2 / x) * (2 / x) * 3.14) <= size and size < 76800:
            if 1000 < max_indices_len:
                if YM['m00'] > 0:
                    rospy.logdebug('Yellow light on')
                    self.light = False
                    cx = int(YM['m10']/YM['m00'])
                    cy = int(YM['m01']/YM['m00'])
                    cv2.circle(image, (cx, cy), 20, (0, 94, 255), -1)

        kernel = numpy.ones((20, 20), numpy.uint8)
        closing = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

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

        cv2.circle(image, (int(circle_x), int(circle_y)), 2 / x, (0, 255, 0), 3)
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)

        if ((2 / x) * (2 / x) * 3.14) <= size and size < 76800:
            if 1000 < max_indices_len:
                if GM['m00'] > 0:
                    rospy.logdebug('green light on')
                    self.light = False
                    cx = int(GM['m10']/GM['m00'])
                    cy = int(GM['m01']/GM['m00'])
                    cv2.circle(image, (cx, cy), 20, (0, 255, 0), -1)



        self.light_pub.publish(self.light)

        cv2.imshow('Red', mask_red )
        cv2.imshow('Yellow', mask_yellow )
        cv2.imshow('Green', mask_green )
        cv2.imshow('image', image)
        cv2.waitKey(3)


if __name__ == "__main__":
    rospy.init_node('light',log_level=rospy.INFO)
    light = Light()
    rospy.spin()
