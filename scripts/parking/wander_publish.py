#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class parking_publish:
  def __init__(self):
    #class init
    self.key_pub = rospy.Publisher('tb3/control/parking_white', Bool, queue_size=1)
    rospy.init_node("test_driver")
    self.rate = rospy.Rate(100)

  def create_parking_publish(self):
    #real function part
    self.key_pub.publish(True)
    self.rate.sleep()

if __name__ == '__main__':
  pubs = parking_publish()
  while not rospy.is_shutdown():
    pubs.create_parking_publish()
  
