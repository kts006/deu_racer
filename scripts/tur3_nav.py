#!/usr/bin/env python
# BEGIN ALL
#-*- coding: utf-8 -*-
import rospy, cv2, cv_bridge
import numpy as np0
from geometry_msgs.msg import Twist
from deu_racer.msg import Deu_pose
from std_msgs.msg import Bool

from elapsed_time import ElapsedTime
import sys
#import pyturtlebot as Robot
vel_scale = 0.22

class Deu_turtlebot:
    def __init__(self):
        self.pose_msg = Deu_pose()
        self.xpose = 160
        self.ypose = 160
        self.control_state = False
        self.nav_state = False
        self.p_state = False
        self.parking_white = False
        
        #self.linear_x = 0.22
        #self.angular_z = 0.0
        
        self.timer = ElapsedTime()
        self.twist = Twist()
        self.control_sub = rospy.Subscriber('tb3/control/block',Bool, self.control_callback)
        self.navmode_sub = rospy.Subscriber('tb3/control/navmode',Bool, self.navmode_callback)
        self.parking_sub = rospy.Subscriber('tb3/control/parking',Bool, self.parking_callback)
        self.parking_white_sub = rospy.Subscriber('/tb3/control/parking_white',Bool, self.parking_white_callback)     
        self.pose_sub = rospy.Subscriber('/line_pose',Deu_pose, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)

    def parking_white_callback(self, msg):
        self.parking_white = msg.data

    def parking_callback(self, msg):
        self.p_state = msg.data
        if not self.p_state :
            self.parking_white = False
        #if self.p_state :
        #    vel_scale = 0.05
        #    print "vel_scale : ", vel_scale
        #else:
        #   vel_scale = 0.22
        #    print "vel_scale : ", vel_scale
          
    def navmode_callback(self, msg):
        self.nav_state = msg.data

    def control_callback(self, msg):
        self.control_state = msg.data

    def pose_callback(self, msg):
#        rospy.logdebug('%d %d nav %s  con %s',msg.xpose, msg.ypose,self.nav_state,self.control_state)
        self.xpose = msg.xpose
        self.ypose = msg.ypose
        #self.navigate()

    def navigate(self):
        if not self.nav_state :
          if not self.control_state :
            if not self.parking_white :
                rospy.logdebug('Pose(%d,%d)',self.xpose, self.ypose)
                err = self.xpose - 320/2
                self.twist.linear.x = vel_scale
                self.twist.angular.z = -float(err)/20 # / 40
                self.cmd_vel_pub.publish(self.twist)

rospy.init_node('deu_turtlebot3_nav',log_level=rospy.INFO)
deutur = Deu_turtlebot()
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    deutur.navigate()
    rate.sleep()
    
