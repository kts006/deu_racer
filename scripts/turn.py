#! /usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import pyturtlebot as Robot

rospy.init_node('turn')

robot = Robot.Move_controller()
robot.turn_tb3(-1.57)
