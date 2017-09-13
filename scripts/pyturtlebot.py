#!/usr/bin/env python
#-*-coding:utf-8-*-
# 참조 : https://github.com/wjwwood/pyturtlebot

import sys
import time
import numpy as np
import random
import tf


from math import radians

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations as trans
from std_msgs.msg import Bool

class Move_controller():
    max_linear = 1.0
    max_angular = 2.0

    def __init__(self):

        rospy.myargv(argv=sys.argv)
        #로봇을 움직이기 위한 필요한 변수들
        self.__x = None
        self.__y = None
        self.__angle = None
        self.__cumulative_angle = 0.0
        self.__have_odom = False
        
        self.tf_listener = tf.TransformListener()
        
        #라이더가 장애물을 발견하고 발견한 횟수가 5번을 넘어가면 확실히 장애물을 인식한 것으로 취급
        self.__ridar_cnt = 0

        self.on_bumper = None
        self.movement_enabled = True
        
        #주차중에 장애물 여부를 확인하기 위한 변수
        self.chk_obstacle = False

        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)
        self.__cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
    def move(self, linear=0.0, angular=0.0):
        """Moves the robot at a given linear speed and angular velocity

        The speed is in meters per second and the angular velocity is in radians per second

        """
        self.__exit_if_movement_disabled()
        # Bounds checking
        if abs(linear) > self.max_linear:
            linear = self.max_linear if linear > self.max_linear else linear
            linear = -self.max_linear if linear < -self.max_linear else linear
        if abs(angular) > self.max_angular:
            angular = self.max_angular if angular > self.max_angular else angular
            angular = -self.max_angular if angular < -self.max_angular else angular
        # Message generation
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        # Announce and publish
        self.__cmd_vel_pub.publish(msg)

    def move_distance(self, distance, velocity=0.05):
        """Moves a given distance in meters

        You can also give it a speed in meters per second to travel at:

            robot.move_distance(1, 0.5)  # Should take 2 seconds
        """
        
        print "move_distance entered"
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            r.sleep()

        msg = Twist()
        msg.linear.x = velocity
        x0 = self.__x
        y0 = self.__y
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            d = ((self.__x - x0)**2 + (self.__y - y0)**2)**0.5
            if d >= distance:
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.linear.x = 0.0
        self.__cmd_vel_pub.publish(msg)
        
    def turn_tb3(self, angle = -1.57):
        print "trun======================================================"
        tb3_twist = Twist()
        is_fisrt = True
        init_yaw = 100.0
        cur_yaw = 100.0
        
        acc_yaw = 0.02
        prev_yaw = cur_yaw
        
        rate = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/odom','/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                print ex
                continue
            
            euler = tf.transformations.euler_from_quaternion(rot)
            if is_fisrt:
                init_yaw = euler[2]
                cur_yaw = init_yaw
                prev_yaw = cur_yaw
                is_fisrt = False
            cur_yaw = euler[2]
            
            
            diff_yaw = init_yaw - cur_yaw
            
            if cur_yaw * prev_yaw < 0 and cur_yaw > 0:
                acc_yaw += abs(cur_yaw + prev_yaw)
            else:
                acc_yaw += abs(cur_yaw - prev_yaw)
            print "cur_yaw", cur_yaw, "prev_yaw", prev_yaw, "acc_yaw", acc_yaw

            if acc_yaw >= abs(angle):
                print "acc_yaw", acc_yaw
                twist = Twist()
                self.__cmd_vel_pub.publish(twist)
                break
                
                
            tb3_twist.angular.z = -0.5
            self.__cmd_vel_pub.publish(tb3_twist)
            
            prev_yaw = cur_yaw  # L
            
            rate.sleep()
        print "init_yaw ",init_yaw,"cur_yaw ", cur_yaw ,"diff_yaw",init_yaw - cur_yaw       
                
    def turn_angle(self, angle, velocity=0.22):
        """Turns the robot a given number of degrees in radians

        You can easily convert degress into radians with the radians() function:

            robot.turn_angle(radians(45))  # Turn 45 degrees

        You can also give an angular velocity to turn at, in radians per second:

            robot.turn_angle(radians(-45), radians(45))  # Turn back over a second
        """
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            r.sleep()

        msg = Twist()
        if angle >= 0:
            msg.angular.z = np.abs(velocity)
        else:
            msg.angular.z = -np.abs(velocity)
        angle0 = self.__cumulative_angle
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            a_diff = self.__cumulative_angle - angle0
            if (angle > 0 and a_diff >= angle) or (angle < 0 and a_diff <= angle):
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)

    def stop(self):
        """Stops the robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.__cmd_vel_pub.publish(msg)

    def wait(self, seconds):
        """This function will wait for a given number of seconds before returning"""
        time.sleep(seconds)

    
    def __odom_handler(self, msg):
        self.__x = msg.pose.pose.position.x
        self.__y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        a = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        # cumulative angle doesn't wrap. assumes we've not moved more than pi radians
        # since last odom message
        if self.__have_odom:
            a_diff = a - self.__angle
            if a_diff > np.pi:
                a_diff -= 2*np.pi
            elif a_diff < -np.pi:
                a_diff += 2*np.pi
            self.__cumulative_angle += a_diff

        self.__angle = a
        self.__have_odom = True
        
    #로봇기준의 범위 내에 장애물이 존재하는지 확인(__Scan_handler에서 발생)
    def is_ac(self, msg):
      range_len = len(msg.ranges[181:325])
      range_ahead = msg.ranges[181:325]

      for i in range(0,range_len-1) :
        if 0.15 <= round(range_ahead[i],1) <=0.45 : 
          if (245-range_len) <= i <= (315-range_len) :
              return True

    #라이다 장애물 확인
    def __scan_handler(self, msg):
         
          if self.is_ac(msg) :
            self.chk_obstacle = True
            self.__ridar_cnt = self.__ridar_cnt + 1
          else:
            self.chk_obstacle = False
            
          if self.__ridar_cnt > 5:
            self.chk_obstacle = True
          #print self.chk_obstacle
          
    #움직임이 제어가 되지 않을 때 발생
    def __exit_if_movement_disabled(self):
        if not self.movement_enabled:
            self.say("Movement currently disabled")
            sys.exit()

    #주차(Parking) 움직임
    def parking_moving(self):
          self.wait(2.0)#not to slip
          self.move_distance(0.35)#forward
          self.wait(0.6)#not to slip

          #장애물이 존재하는지 /scan 토픽을 이용하여 확인
          if self.chk_obstacle :
            self.stop_behavior()
          else:
            self.move_command()
          self.__ridar_cnt = 0
    
    #장애물이 없을 때 주차행동
    def move_command(self):
        self.turn_tb3(-1.57)
        #self.turn_angle(radians(-75),radians(-75))#90
        self.wait(0.6)

        self.move_distance(0.25)#forward aa
        self.wait(2.0)#wait for stay(2s waiting)

        self.turn_tb3(-3.1415)
        #self.turn_angle(radians(150), radians(150))#180
        self.wait(0.6)#not to slip

        self.move_distance(0.25)#forward aa
        self.wait(0.6)
        
        self.turn_tb3(-1.57)
        #self.turn_angle(radians(-75), radians(-75))
        self.wait(0.6)

    # 장애물을 만났을 때 해야 할 행동
    def stop_behavior(self):
         
        self.move_distance(0.4)#forward
        
        self.move_command()
          
