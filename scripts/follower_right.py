#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from deu_racer.msg import Deu_pose


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.pose_msg = Deu_pose()
        self.pose_pub = rospy.Publisher('/right_pose',Deu_pose,queue_size = 1)
        self.image_sub = rospy.Subscriber('/front_right_camera/image_raw/compressed', CompressedImage, self.image_callback)
        
    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')
        
        rows,cols,ch = image.shape
        pts1 = np.float32([[0,50],[90,50],[0,90],[160,90]])
        pts2 = np.float32([[0,0],[160,0],[0,120],[160,120]])
        M = cv2.getPerspectiveTransform(pts1,pts2)
        dst = cv2.warpPerspective(image,M,(160,120))        
       
        hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 0,  0,  240])
        upper_yellow = np.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = h/2
        search_bot = search_top + 30
        mask[0:search_top, 0:w]=0
        mask[search_bot:h, 0:w]=0
        M = cv2.moments(mask)
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.pose_msg.xpose = cx
            self.pose_msg.ypose = cy
            self.pose_pub.publish(self.pose_msg)
            image= cv2.circle(image, (cx, cy), 5, (0,0,255), -1)

        cv2.imshow("Rdst", dst)
        cv2.imshow("Rimage", image)
        cv2.imshow("Rleft", mask)
        cv2.waitKey(3)
        
rospy.init_node('R_follower')
follower = Follower()
rospy.spin()
# END ALL
