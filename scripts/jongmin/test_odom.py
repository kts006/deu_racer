#!/usr/bin/env python

import rospy
import pyturtlebot
import numpy as np
import math

robot = pyturtlebot.get_robot()
#robot.turn_angle(radians(90))
robot.move_distance(0.1)
robot.turn_angle(math.radians(78), math.radians(78))
robot.wait(0.6)
robot.move_distance(0.1)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    r.sleep()

