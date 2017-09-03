#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2, cv_bridge
import numpy as np
from matplotlib import pyplot as plt

FLANN_INDEX_KDTREE = 0
MIN_MATCH_COUNT=10

sumlist = []
listdata = []
MAX_LIST_SIZE = 10

class Match:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #self.sift = cv2.xfeatures2d.SIFT_create()
        self.surf = cv2.xfeatures2d.SURF_create()
        self.queryImg = cv2.imread('jpg/outBtn.jpg',0)  # queryImage
        self.queryKp, self.queryDes = self.surf.detectAndCompute(self.queryImg,None)
        self.res = None
        self.flannParam=dict(algorithm=FLANN_INDEX_KDTREE,tree=5)
        self.flann=cv2.FlannBasedMatcher(self.flannParam,{})
        
    def matching(self,trainImg):
        trainKp, trainDes = self.surf.detectAndCompute(trainImg,None)
        
        matches=self.flann.knnMatch(trainDes,self.queryDes,k=2)
        goodMatch=[]
        print matches
        for m,n in matches:
            if(m.distance<0.7*n.distance):
                goodMatch.append(m)
                
        if(len(goodMatch)>MIN_MATCH_COUNT):
            tp=[]
            qp=[]
            for m in goodMatch:
                tp.append(self.queryKp[m.trainIdx].pt)
                qp.append(trainKp[m.queryIdx].pt)
            tp,qp=np.float32((tp,qp))
            H,status=cv2.findHomography(tp,qp,cv2.RANSAC,3.0)
            h,w=self.queryImg.shape
            trainBorder=np.float32([[[0,0],[0,h-1],[w-1,h-1],[w-1,0]]])
            queryBorder=cv2.perspectiveTransform(trainBorder,H)
            
            #print np.int32(queryBorder)
            #np.int32(queryBorder[0][0]) = np.int32(addmatchlist(np.int32(queryBorder[0][0]))
            cv2.polylines(trainImg,[np.int32(queryBorder)],True,(255,0,0),5)
        else:
            print "Not Enough match found- %d/%d"%(len(goodMatch),MIN_MATCH_COUNT)
                        
        return trainImg
        
    def addmatchlist(self,list):
        if len(listdata)>MAX_LIST_SIZE:
            listdata.pop(0)
        else :
            listdata.append(list)
                    
    def averagemathclist(self,points):
        list = [[0,0],[0,0]]
        for i in range(len(listdata)):
            position = 0
            for m,n in listdata[i]:
                list[position][0] += m
                list[position][1] += n
                position = 1

        i = 0
        for m,n in list:
            print m,n
            list[i][0] = m /4
            list[i][1] = n/4
            i += 1

        return list
        

    
        
        
