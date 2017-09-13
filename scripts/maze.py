#!/usr/bin/env python
#-*-coding:utf-8-*-
import time
import rospy
import math
import cv2, cv_bridge
import numpy as np
import pyturtlebot
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class maze_Solver:
    def __init__(self):
        
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.tunnel_sub = rospy.Subscriber('/tb3/control/tunnel',Bool,  self.tunnel_callback)
        self.tunnelend_pub = rospy.Publisher('/tb3/conrol/tunnel_end',Bool,queue_size = 1)
        self.tunnel_state = False
        self.move_distance = 0.2
        self.maze_size = 10
        self.scan_ahead = 10.000
        self.scan_left = 10.000
        self.scan_right = 10.000
        self.pose_list = []
        self.turn_count = 0
        self.maze = np.array([[0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0],
                              [0,0,0,0,0,0,0,0,0,0]])
        self.robot = pyturtlebot.Move_controller()
        
    
    def scan_callback(self, msg):
        Rscan_ahead = min(msg.ranges[0:5])
        Lscan_ahead = min(msg.ranges[355:359])
        self.scan_ahead= min(Rscan_ahead,Lscan_ahead)
        self.scan_left = min(msg.ranges[85:95])
        self.scan_right = min(msg.ranges[265:275])
        #rospy.logdebug('%s %s %s', self.scan_ahead,self.scan_left, self.scan_right)
        
    def tunnel_callback(self, msg):
        self.tunnel_state = msg.data
        
    def printSolution(self):
        print self.maze
        print self.pose_list
        self.tunnelend_pub.publish(True)
            
    def isSafe(self, x, y):
        rospy.logdebug('%s %s %s', self.scan_ahead,self.scan_left, self.scan_right)
        rospy.logdebug('Is safe %d, %d, %d',x,y,self.maze[x][y] == -1)  
        return (x>=0 and y>=0 and x<self.maze_size and y<self.maze_size and not (self.maze[x][y] == -1))
        
    def solveMaze(self):
        rospy.logdebug('maze solve')
        if (self.solveMazeUtil(9, 9) == False):
            return False
            
        self.printSolution()
        return True
        
    def solveMazeUtil(self, x, y):
        rospy.logdebug('solveMazeUtil %d %d',x,y)
        self.search(x,y)
        
        if(x == 0 and y == 0):
            self.maze[x][y] = 1;
            return True
            
        if(self.isSafe(x, y) == True):
            rospy.logdebug('isSafe True')
            self.maze[x][y] = 1;
                        
            ## 로봇움직임, 현재좌표를 기억'
            if not(x == self.maze_size-1 and y == self.maze_size-1):
                self.move(x,y)
            self.pose_list.append([x,y])    

            if (self.solveMazeUtil(x - 1, y)): #ahead
                return True;
            
            if (self.solveMazeUtil(x, y - 1)): #left
                return True;
                                
            if (self.solveMazeUtil(x, y + 1)):#right
                return True;
                
            self.maze[x][y] = -1;
            #이전좌표로 이동
            self.pose_list.pop()
            move_x,move_y = self.pose_list[len(self.pose_list)-1]
            self.move(move_x,move_y)
            return False;
            
        return False
    
    def move(self, x, y):
        
        cur_x,cur_y = self.pose_list[len(self.pose_list)-1]
        rospy.logdebug('move to %d, %d cur x %d y %d, turn_count %d',x,y,cur_x,cur_y,self.turn_count)
        if x == cur_x and y == cur_y-1 :
            if self.turn_count == 0:
                self.turn(1)
                self.go(self.move_distance)
            elif self.turn_count == 1:            
                self.go(self.move_distance)
            elif self.turn_count == 2:
                self.turn(-1)
                self.go(self.move_distance)
            elif self.turn_count == 3:
                self.turn(2)
                self.go(self.move_distance)

        elif x == cur_x-1 and y == cur_y :
            if self.turn_count == 0:
                self.go(self.move_distance)
            elif self.turn_count == 1:
                self.turn(-1)
                self.go(self.move_distance)
            elif self.turn_count == 2:
                self.turn(2)
                self.go(self.move_distance)
            elif self.turn_count == 3:
                self.turn(1)
                self.go(self.move_distance)
            
        elif x == cur_x and y == cur_y+1 :
            if self.turn_count == 0:
                self.turn(-1)
                self.go(self.move_distance)
            elif self.turn_count == 1:
                self.turn(2)
                self.go(self.move_distance)
            elif self.turn_count == 2:
                self.turn(1)
                self.go(self.move_distance)
            elif self.turn_count == 3:
                self.go(self.move_distance)
            
        elif x == cur_x+1 and y == cur_y :
            if self.turn_count == 0:
                self.turn(2)
                self.go(self.move_distance)
            elif self.turn_count == 1:
                self.turn(1)
                self.go(self.move_distance)
            elif self.turn_count == 2:
                self.go(self.move_distance)
            elif self.turn_count == 3:
                self.turn(-1)
                self.go(self.move_distance)
            
    def go(self,distance):
        self.robot.move_distance(distance)
        self.robot.wait(1)
        
    def turn(self,direction):
        self.robot.turn_angle(math.radians(direction*78), math.radians(direction*78))
        self.robot.wait(1)
        if direction == -1:
            self.turn_count = self.turn_count + 1
        elif direction == 1:
            self.turn_count = self.turn_count - 1
        elif direction == 2:
            self.turn_count = self.turn_count + 1
            
        self.turn_count = abs(self.turn_count%4)
    
    def map_make(self, x, y): #좌표맵상의 장애물 표시
        if not(x>=self.maze_size or y>=self.maze_size or x<0 or y<0):
            rospy.logdebug('Map MAKE %d, %d',x,y)
            self.maze[x][y] = -1

    def search(self, x, y):  ## 라이다 /scan데이터 이용 장애물 확인
        rospy.logdebug('search %d, %d',x,y)
        rospy.logdebug('%f %f, %f',self.scan_ahead, self.scan_right, self.scan_left)
        
        if self.turn_count == 0:
            if self.scan_ahead <= 0.2:
                self.map_make(x,y-1)
            if self.scan_right <= 0.2:
                self.map_make(x+1,y)
            if self.scan_left <= 0.2:
                self.map_make(x-1,y)
        
        elif self.turn_count == 1:
            if self.scan_ahead <= 0.2:
                self.map_make(x-1,y)
            if self.scan_right <= 0.2:
                self.map_make(x,y-1)
            if self.scan_left <= 0.2:
                 self.map_make(x,y+1)
        
        elif self.turn_count == 2:
            if self.scan_ahead <= 0.2:
                 self.map_make(x,y+1)
            if self.scan_right <= 0.2:
                 self.map_make(x-1,y)
            if self.scan_left <= 0.2:
                 self.map_make(x+1,y)
                
        elif self.turn_count == 3:
            if self.scan_ahead <= 0.2:
                 self.map_make(x+1,y)
            if self.scan_right <= 0.2:
                 self.map_make(x,y-1)
            if self.scan_left <= 0.2:
                 self.map_make(x,y+1)

if __name__ == "__main__":
    rospy.init_node('maze',log_level=rospy.DEBUG)
    maze_sol = maze_Solver()
    #while True:
    #    if maze_sol.tunnel_state == True:
    #        maze_sol.go(0.3)
    #        maze_sol.solveMaze()
    maze_sol.go(0.3)
    maze_sol.solveMaze()
    rospy.spin()
   
