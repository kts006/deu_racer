#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
import tf

if __name__== '__main__':
    rospy.init_node('tb3_tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
    
        try:
            (trans,rot) = listener.lookupTransform('/odom','/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        euler = tf.transformations.euler_from_quaternion(rot)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        
        print trans, yaw
        
        rate.sleep()
