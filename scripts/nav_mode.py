#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, cv2, cv_bridge
import sys, select, termios, tty
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

msg = """
Choice turtlebot3 navigation mode!!
---------------------------------------
Navigation Mode :
 'r' - RUN Mode (Navigation mode)
 'p' - PAUSE Mode (User control mode)
---------------------------------------
           Moving around :
                 w
            a    s    d  
                 x
                 
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
"""

class Nav_Mode :
    def __init__(self):
        rospy.loginfo('%s', msg)
        self.nav_status = True
        self.target_linear_vel = 0
        self.target_angular_vel = 0
        self.control_linear_vel = 0
        self.control_angular_vel = 0
        self.twist = Twist()
        self.mode_pub = rospy.Publisher('tb3/control/navmode', Bool, queue_size=5)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        rospy.loginfo("currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel))
        
    def key_publisher(self):
        try:
            while(1):
                key = self.getKey()
                # Navigation Mode
                if key == 'r' :
                    self.nav_status = False
                    self.target_linear_vel   = 0
                    self.control_linear_vel  = 0
                    self.target_angular_vel  = 0
                    self.control_angular_vel = 0
                    rospy.loginfo('RUN MODE')
                    #self.mode_pub.publish(self.nav_status)
                
                # Pause Mode    
                elif key == 'p' :
                    self.nav_status = True
                    rospy.loginfo('PAUSE MODE')
                else:
                    if (key == '\x03'):
                        break
                    
                if self.nav_status:
                    if key == 'w' :
                        self.target_linear_vel = self.target_linear_vel + 0.01
                        self.vels(self.target_linear_vel,self.target_angular_vel)
                        
                    elif key == 'x' :
                        self.target_linear_vel = self.target_linear_vel - 0.01
                        self.vels(self.target_linear_vel,self.target_angular_vel)
                        
                    elif key == 'a' :
                        self.target_angular_vel = self.target_angular_vel + 0.1
                        self.vels(self.target_linear_vel,self.target_angular_vel)
                        
                    elif key == 'd' :
                        self.target_angular_vel = self.target_angular_vel - 0.1
                        self.vels(self.target_linear_vel,self.target_angular_vel)
                        
                    elif key == ' ' or key == 's' :
                        self.target_linear_vel   = 0
                        self.control_linear_vel  = 0
                        self.target_angular_vel  = 0
                        self.control_angular_vel = 0
                        self.vels(0, 0)
                    
                    if self.target_linear_vel > self.control_linear_vel:
                        self.control_linear_vel = min( self.target_linear_vel, self.control_linear_vel + (0.01/4.0) )
                    else:
                        self.control_linear_vel = self.target_linear_vel

                    if self.target_angular_vel > self.control_angular_vel:
                        self.control_angular_vel = min( self.target_angular_vel, self.control_angular_vel + (0.1/4.0) )
                    else:
                        self.control_angular_vel = self.target_angular_vel

                
                    self.twist.linear.x = self.control_linear_vel; self.twist.linear.y = 0; self.twist.linear.z = 0
                    self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = self.control_angular_vel
                    self.twist_pub.publish(self.twist)
                    
                self.mode_pub.publish(self.nav_status)

        finally:
            twist = Twist()
            self.twist_pub.publish(twist)
            self.mode_pub.publish(self.nav_status)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('deu_turtlebot3_navMode',log_level=rospy.INFO)
    nav_mode = Nav_Mode()
    nav_mode.key_publisher()
    
    
