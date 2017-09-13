#!/usr/bin/env python  
#-*- coding: utf-8 -*-

# http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29


import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tb3_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        
        print trans, yaw
        rate.sleep()
		

		
# 참고: https://answers.ros.org/question/69754/quaternion-transformations-in-python/


