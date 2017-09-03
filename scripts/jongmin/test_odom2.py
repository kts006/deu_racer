#!/usr/bin/evn python
#-*-coding:utf-8-*-
import rospy
import pyturtlebot
import numpy as np
from math import radians

#degrees -> radians에 오차에 있어서 90도가 대략 72정도 된다.
#좀 더 정확하게는 radians를 이용하지 않고 90도는 1.57 radian이다.
robot = pyturtlebot.get_robot()

robot.move_distance(0.1)#forward
#robot.wait(0.6)#not to slip

robot.turn_angle(radians(78),radians(78))#90
robot.wait(0.6)

#이 쯤에서 장애물 체크
#장애물이 없다면 아래를 실행

robot.move_distance(0.3)#forward
robot.wait(2.0)#wait for stay(2s waiting)

robot.turn_angle(radians(158), radians(158))#180
robot.wait(0.6)#not to slip

robot.move_distance(0.3)#forward
#robot.wait(0.6)

robot.turn_angle(radians(78), radians(78))
robot.wait(0.6)

#여기서 부터는 다시 주행모드로 들어가는게 나을 듯#
#robot.move_distance(0.2)
#주차가 끝난다.

r = rospy.Rate(100)
while not rospy.is_shutdown():
  r.sleep()
