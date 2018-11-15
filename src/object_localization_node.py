#! /usr/bin/env python

#Modules import
import rospy
import numpy as np
import tf
import datetime

from math import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from darknet_ros_msgs.msg import *
from operator import attrgetter

#==============================================================================
# class localize():
#     def __init__(self):
#         rospy.init_node('object_localization_node')
# 
#         print "object_localization_node initialized"
#==============================================================================

#==============================================================================
# 
#         rate = rospy.Rate(2)
#         while not rospy.is_shutdown():
#             print "loop is running"
#             rate.sleep()
#==============================================================================

#==============================================================================
# This code does the following:
#  It subscribes to /darknet_ros/bounding_boxes topic
#  It gets the bounding box around the mannequin with the highest confidence
#  It converts pixel coordinates of bounding box center to meters
#  It reads from the ZED camera the depth of the object
#  It listens to the TF transform, and it transforms
#              (x,y,z) in meters from the camera frame to the map frame
#  It publishes the (x,y,z) in the map frame so the drone knows where to go
#==============================================================================

# ZED M left camera matrix  (pixel units) - obtained from /zed/left/camera_info
# message type is sensor_msgs/CameraInfo
K = [677.0139770507812, 0.0, 635.8870239257812, 0.0, 677.0139770507812, 
     373.62017822265625, 0.0, 0.0, 1.0]
fx = K[0] #focal length along the x-axis
fy = K[4] #focal length along the y-axis
cx = K[2] #principal point - x coordinate
cy = K[5] #principal point - y coordinate



def bounding_boxes_cb(data):
    a = sum(p.Class == "mannequin/person" for p in data.bounding_boxes)
    if a > 0:
        print "confidence = ", max(data.bounding_boxes, key=lambda x: x.probability)
    for i in range(len(data.bounding_boxes)):
        if data.bounding_boxes[i].Class == "mannequin/person":
            print "found missing worker at time {}".format(rospy.Time.now())

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes,
                     bounding_boxes_cb)
    
    
    rospy.spin()

def pix2meters(x,y,Z):
#==============================================================================
#     x and y are the center coordinates of the bounding box
#     Z is the depth at x and y
#     This function returns x and y converted from pixels to meters
#                in the camera frame using the camera matrix
#==============================================================================
    X = ((x-cx)*Z)/fx
    Y = ((y-cy)*Z)/fy
    return X,Y


if __name__ == '__main__':
    try:
        print "main"
        listener()
    except rospy.ROSInterruptException:
        print "exit"
        
        
