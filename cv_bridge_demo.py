#!/usr/bin/env python
# coding:utf-8

""" cv_bridge_demo.py - Version 1.1 2013-12-20
    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import time
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
def nothing(a):
    pass

CPM=0
def f_d(d):
    global CPM
    return np.exp(-((CPM-d)/255*10)**2)*(math.cos((CPM-d)/18))**2
def transf(d):
    global CPM
    return np.multiply(np.exp(-np.multiply((CPM-d)/255*10,(CPM-d)/255*10)),np.cos(((CPM-d)/18))**2)

class cvBridgeDemo():
    def __init__(self):
        plt.axis([0, 255, 0, 1])
        plt.xlabel("depth")
        plt.ylabel("weight")
        plt.ion()
	classfier = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
	color = (0, 255, 0)
        self.node_name = "zhanshi"
        self.color_image = cv2.imread("/home/ryc/pic.png")
        self.depth_image = cv2.imread("/home/ryc/picd.png",0)
        rospy.init_node(self.node_name)
        rospy.on_shutdown(self.cleanup)
        cv2.namedWindow('deep',0)
        cv2.namedWindow('img_t',0)
        cv2.namedWindow('img',0)
        cv2.namedWindow('central_principal_maximum_editor',0)

        cv2.createTrackbar('CPM','central_principal_maximum_editor',0,255,nothing)
        cv2.moveWindow("central_principal_maximum_editor", 400, 700);
        cv2.moveWindow("deep", 0, 0);
        cv2.moveWindow("img_t", 650, 0);
        cv2.moveWindow("img", 1280, 0);
        cv2.resizeWindow("deep",600,470)
        cv2.resizeWindow("img_t",600,470)
        cv2.resizeWindow("img",600,470)
            # Create the OpenCV display window for the RGB image
        
        #cv.NamedWindow("Depth Image", cv.CV_WINDOW_NORMAL)
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks

        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback, queue_size=1)
        
        while(1):
            
            global CPM
            img=self.color_image

            imgd=self.depth_image
           
           
            x= np.linspace(1,255,255)
            y= np.zeros(x.shape)
            CPM=cv2.getTrackbarPos('CPM','central_principal_maximum_editor')
            #print(CPM)
            img_transformed=np.zeros(img.shape)
            fd=np.zeros(imgd.shape)
            fd3=np.zeros(img.shape)
            #print(imgd[100][100])
            imgd=imgd.astype('float64')
            cv2.normalize(imgd, imgd, 0, 255, cv2.NORM_MINMAX)
            #print(imgd[100][100])
            fd=transf(imgd)
            imgd=imgd.astype('uint8')
            cv2.imshow('deep',imgd)
            p = cv2.waitKey(1)
            if(p == 1048689):#if press q, quite
                cleanup(self)
                break
            for i in range(3):
                fd3[:,:,i]=fd
            #fd3=cv2.blur(fd3,(7,7))
            img_transformed=np.multiply(img,fd3)

            img_transformed=img_transformed.astype('uint8')
            x_=[]
            y_=[]
            w=[]
            h=[]
            s=[]
            #print type(x_)
            faceRects = classfier.detectMultiScale(img_transformed, scaleFactor = 1.2, minNeighbors = 3, minSize = (32, 32))
            if len(faceRects) > 0:          #face>0      
                                                                   
                for i,faceRect in enumerate(faceRects):  #rect a face
                    x_.append(0)
                    y_.append(0)
                    w.append(0)
                    h.append(0)

                    x_[i], y_[i], w[i], h[i] = faceRect
                    s.append(w[i]*h[i])
                ind=0
                max=s[0]
                for i in range(len(s)):
                    if s[i]>max:
			ss=max 
			ind=i
		cv2.rectangle(img_transformed, (x_[ind], y_[ind]), (x_[ind] + w[ind] , y_[ind] + h[ind]), color, 2)	
            x_=[]
            y_=[]
            w=[]
            h=[]
            s=[]
            faceRects = classfier.detectMultiScale(img, scaleFactor = 1.2, minNeighbors = 3, minSize = (32, 32))
            if len(faceRects) > 0:          #face>0      
                                                                   
                for i,faceRect in enumerate(faceRects):  #rect a face
                    x_.append(0)
                    y_.append(0)
                    w.append(0)
                    h.append(0)

                    x_[i], y_[i], w[i], h[i] = faceRect
                    s.append(w[i]*h[i])
                ind=0
                max=s[0]
                for i in range(len(s)):
                    if s[i]>max:
			ss=max 
			ind=i
		cv2.rectangle(img, (x_[ind], y_[ind]), (x_[ind] + w[ind] , y_[ind] + h[ind]), color, 2)
            for i in range(255):
                y[i]=f_d(x[i])
            plt.cla()
            plt.plot(x,y,'-')
            plt.axis([0, 255, 0, 1])
            plt.show()
            plt.pause(0.05)
            cv2.imshow('img',img)
            p = cv2.waitKey(1)
            if(p == 1048689):#if press q, quite
                cleanup(self)
                break
            cv2.imshow('img_t',img_transformed)
            p = cv2.waitKey(1)

            if(p == 1048689):#if press q, quite
                cleanup(self)
                break
            #print(self.depth_image[300][300])
           
            '''
            cv2.imshow('color',img)
            cv2.waitKey(10)
            cv2.imshow('deep',imgd)
            cv2.waitKey(10)
            '''
            
    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
        #cv2.imshow("color_image",self.color_image)
        #cv2.waitKey(10)
        
                
    def depth_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            self.depth_image = self.bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e
        #self.depth_image = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #print(self.depth_image)
        #cv2.imshow("Depth_Image",self.depth_image)
        #cv2.waitKey(10)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   

def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
