#!/usr/bin/env python
#-*-coding:utf-8-*-
#parking bool을 읽어서 
import rospy
import math
import numpy as np
import pytur as Robot
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

if __name__ == "__main__":
  while not rospy.is_shutdown():
    parking = Robot.Turtlebot()
    rospy.spin()

