#=====================================================
# Application and Practice of Autonomous Mobile Robots
# Copyright 2025 ASRLAB
#=====================================================

#!/usr/bin/env python3
# coding=utf-8

# Import necessary modules for ROS 2 communication
import os  # Operating system module
import select  # I/O multiplexing module
import sys  # System-specific parameters and functions module
import rclpy  # ROS 2 client library
import time
import math
import cv2
import cv_bridge
import numpy as np
import numpy
import json

# Import message types and Quality of Service (QoS) profile for ROS 2
from rclpy.qos import QoSProfile  # Quality of Service profile for ROS 2
from rclpy.node import Node
from sensor_msgs.msg import Image


def nothing(s):
    pass
    

def image_callback(msg):
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    lowerbH=cv2.getTrackbarPos("lowerbH",'Adjust_hsv')
    lowerbS=cv2.getTrackbarPos("lowerbS",'Adjust_hsv')
    lowerbV=cv2.getTrackbarPos("lowerbV",'Adjust_hsv')
    upperbH=cv2.getTrackbarPos("upperbH",'Adjust_hsv')
    upperbS=cv2.getTrackbarPos("upperbS",'Adjust_hsv')
    upperbV=cv2.getTrackbarPos("upperbV",'Adjust_hsv')
    
    hsv_values = [lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV]
    file_path = os.path.join(os.getcwd(), 'src/object_tracking/object_tracking/hsv_values.json')
    with open(file_path, 'w') as f:
    	json.dump(hsv_values, f)
    
    kernel = numpy.ones((5,5),numpy.uint8)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_erode = cv2.erode(hsv,kernel,iterations=1)
    hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
    mask=cv2.inRange(hsv_dilate,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
    masked_img = cv2.bitwise_and(image, image, mask=mask)
    
    cv2.imshow("TT", masked_img)
    cv2.waitKey(3)

def main():
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('Task2')
    image_sub = node.create_subscription(Image,'/camera/camera/color/image_raw',image_callback,qos)
    
    cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
    cv2.createTrackbar("lowerbH",'Adjust_hsv',0,255,nothing)
    cv2.createTrackbar("lowerbS",'Adjust_hsv',0,255,nothing)
    cv2.createTrackbar("lowerbV",'Adjust_hsv',0,255,nothing)
    cv2.createTrackbar("upperbH",'Adjust_hsv',125,255,nothing)
    cv2.createTrackbar("upperbS",'Adjust_hsv',125,255,nothing)
    cv2.createTrackbar("upperbV",'Adjust_hsv',125,255,nothing)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
