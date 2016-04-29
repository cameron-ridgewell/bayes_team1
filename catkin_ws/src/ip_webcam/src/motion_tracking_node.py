#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import urllib2
import time 
from geometry_msgs.msg import Twist
import cv2
import numpy as np

def callback(data):
	rospy.loginfo(data.data)
	if(len(data.data)>0):
		id = data.data[0];
		objectWidth = data.data[1];
		objectHeight = data.data[2];
		cvHomography = cv2.CreateMat(3, 3, cv2.CV_32F)
		cvHomography[0,0] = data.data[3];
		cvHomography[1,0] = data.data[4];
		cvHomography[2,0] = data.data[5];
		cvHomography[0,1] = data.data[6];
		cvHomography[1,1] = data.data[7];
		cvHomography[2,1] = data.data[8];
		cvHomography[0,2] = data.data[9];
		cvHomography[1,2] = data.data[10];
		cvHomography[2,2] = data.data[11];
		inPts = [[0,0],[objectWidth,0],[0,objectHeight],[objectWidth,objectHeight]]
		outPts = cv2.perspectiveTransform(inPts, cvHomography)
		print outPts
		#pub = rospy.Publisher("/ip_camera_motion", Twist, queue_size=1)

    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('object_tracking', anonymous=True)

    rospy.Subscriber("/objects", Float32MultiArray, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
