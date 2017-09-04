#!/usr/bin/env python
#-*-coding:utf-8-*-
#https://github.com/wjwwood/pyturtlebot

import sys
import time
import numpy as np
import random

from math import radians

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf import transformations as trans

#for following base
import cv_bridge
from sensor_msgs.msg import Image
import cv2
_turtlebot_singleton = None

def get_robot():
    global _turtlebot_singleton
    if _turtlebot_singleton is None:
        _turtlebot_singleton = Turtlebot()
    return _turtlebot_singleton


class Turtlebot(object):
    max_linear = 1.0
    max_angular = 2.0

    def __init__(self):
        rospy.init_node('pyturtlebot')
        rospy.myargv(argv=sys.argv)

        self.__x = None
        self.__y = None
        self.__angle = None
        self.__cumulative_angle = 0.0
        self.__have_odom = False

        self.movement_enabled = True

        self.__cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.__odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_handler)
        self.__scan_sub = rospy.Subscriber('/scan', LaserScan, self.__scan_handler)


    def move(self, linear=0.0, angular=0.0):
        """Moves the robot at a given linear speed and angular velocity

        The speed is in meters per second and the angular velocity is in radians per second

        """
        self.__exit_if_movement_disabled()
        # Bounds checking
        if abs(linear) > self.max_linear:
            self.say("Whoa! Slowing you down to within +/-{0} m/s...".format(self.max_linear))
            linear = self.max_linear if linear > self.max_linear else linear
            linear = -self.max_linear if linear < -self.max_linear else linear
        if abs(angular) > self.max_angular:
            self.say("Whoa! Slowing you down to within +/-{0} rad/s...".format(self.max_angular))
            angular = self.max_angular if angular > self.max_angular else angular
            angular = -self.max_angular if angular < -self.max_angular else angular
        # Message generation
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        # Announce and publish
        self.say("Moving ('{linear}' m/s, '{angular}' rad/s)...".format(linear=linear, angular=angular))
        self.__cmd_vel_pub.publish(msg)

    def move_distance(self, distance, velocity=1.0):
        """Moves a given distance in meters

        You can also give it a speed in meters per second to travel at:

            robot.move_distance(1, 0.5)  # Should take 2 seconds
        """
        self.__exit_if_movement_disabled()
        # No bounds checking because we trust people. Not like William.
        r = rospy.Rate(1)
        while not self.__have_odom and not rospy.is_shutdown():
            self.say("Waiting for odometry")
            r.sleep()

        msg = Twist()
        msg.linear.x = velocity
        x0 = self.__x
        y0 = self.__y
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            d = ((self.__x - x0)**2 + (self.__y - y0)**2)**0.5
            if d >= distance:
                break

            self.__cmd_vel_pub.publish(msg)
            r.sleep()
        msg.linear.x = 0.0
        self.__cmd_vel_pub.publish(msg)
    

    def block_exist(self):
        pass

    def turn_angle(self, angle, velocity=1.0):
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
            self.say("Waiting for odometry")
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
        self.say("Stopping the robot!")
        self.__cmd_vel_pub.publish(msg)

    def wait(self, seconds):
        """This function will wait for a given number of seconds before returning"""
        self.say("Waiting for '{0}' seconds.".format(seconds))
        time.sleep(seconds)

    def say(self, msg):
        """Prints a message to the screen!"""
        print(msg)
        sys.stdout.flush()


    def get_ranges(self):
        return self.current_laser_msg.ranges

    def say_ranges(self):
	msg = self.current_laser_msg
	rngs = msg.ranges
	self.say("range angle_min is {0}".format(msg.angle_min))
	self.say("range angle_max is {0}".format(msg.angle_max))
	self.say("range size is {0}".format(len(rngs)))
	self.say("range min is {0}".format(msg.range_min))
	self.say("range max is {0}".format(msg.range_max))

	for i in range(0, 63):
	    rng_pos = i * 10
	    self.say("Range {0} is {1}".format(rng_pos, rngs[rng_pos]))
	    rng_pos += 5
	    self.say("Range {0} is {1}".format(rng_pos, rngs[rng_pos]))

    def index_to_rad(self, idx):
        msg = self.current_laser_msg
        rng = msg.angle_max
        rlen = len(msg.ranges)
        self.say("rng: " + str(rng))
        self.say("rlen: " + str(rlen))
        return -(rlen / 2.0 - idx) * rng / (rlen / 2.0)
    
    def turn_around(self):
        self.turn_angle(np.pi)

    def random_angle(self):
        return random.uniform(-np.pi,np.pi)

    def turn_random(self):
        self.turn_angle(self.random_angle())

    def find_closest(self):
        msg = self.current_laser_msg
        rngs = msg.ranges
        idx = np.nanargmin(rngs)
        self.say("idx: " + str(idx))                
        rad = self.index_to_rad(idx)
        return rngs[idx], rad

    def point_at_closest(self):
	rng, rad = self.find_closest()
	self.turn_angle(rad)

    def reset_movement(self):
        self.movement_enabled = True

    def show_laser(self):
        from IPython import display
        from pylab import subplot, show

        lm = self.current_laser_msg
        r = lm.ranges
        theta = [radians(90) + lm.angle_min + i * lm.angle_increment for i, x in enumerate(r)]

        ax = subplot(111, polar=True)
        ax.plot(theta, r, color='r', linewidth=3)
        ax.set_rmax(6)
        ax.grid(True)

        ax.set_title("A line plot of the laser data", va='bottom')
        show()

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

    def __scan_handler(self, msg):
        self.current_laser_msg = msg

    def __exit_if_movement_disabled(self):
        if not self.movement_enabled:
            self.say("Movement currently disabled")
            sys.exit()

