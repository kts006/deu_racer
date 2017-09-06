#!/usr/bin/env python
#-*- coding: utf-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge
import numpy as np0
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

## 라인을 못찾았을 경우 제자리에서 회전을 하여 라인을 찾음

#속도 default 값
class Deu_stabilizer:
    def __init__(self):
        self.twist = Twist()
        self.navmode = True
        self.stabilizer = False
        self.rotate_sub = rospy.Subscriber('tb3/control/stabilize',Bool, self.rotate_callback)
        self.navmode_sub = rospy.Subscriber('tb3/control/navmode',Bool, self.navmode_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
        self.tunnel_sub = rospy.Subscriber('/tb3/control/tunnel',Bool,  self.tunnel_callback)
        self.tunnel_end_sub = rospy.Subscriber('/tb3/control/tunnel_end',Bool,  self.tunnel_callback)
        self.tunnel = False
        self.tunnel_end = False
        
    def tunnel_end_callback(self, msg):
        self.tunnel_end = msg.data
    
    def navmode_callback(self, msg):
        self.navmode = msg.data
    
    def tunnel_callback(self, msg):
        if not self.tunnel_end:
            self.tunnel = msg.data
        else :
            self.tunnel = False
    
    def rotate_callback(self, msg):
        self.stabilizer = msg.data
        if (self.stabilizer and not self.navmode):
		if not self.tunnel:
		    self.twist.linear.x = 0
		    self.twist.angular.z = -0.5
		    self.cmd_vel_pub.publish(self.twist)
        
rospy.init_node('deu_turtlebot3_stabilizer',log_level=rospy.DEBUG)
stabilizer = Deu_stabilizer()
rospy.spin()

