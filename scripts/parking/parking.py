
class Parking:
    def __init__(self):
      self.p_state = False
      
      self.parking_pub = rospy.Publisher('/tb3/control/parking',Bool)
      
    def parking_move(self):
          robot = Robot.Move_controller()
          robot.parking_moving()
    
rospy.init_node('deu_turtlebot3_parking',log_level=rospy.DEBUG)
p = Parking()
rospy.spin()


    def __bool_handler(self, msg):
          self.__parking_start = msg.data
          if self.__parking_start:
            self.cnt_p = self.cnt_p + 1
            if self.cnt_p < 2:
                self.control_pub.publish(True)
                self.parking_move()
            else :
                #after end of this, should turn off
                self.__parking_start = False
          rospy.logdebug('parking_start state is %s',self.__parking_start)
          
          

