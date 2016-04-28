#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import time 
import cv2
from geometry_msgs.msg import Twist
import sys

def sendTwist(twistX, twistY):
    twist = Twist()
    twist.linear.x = twistX
    twist.linear.y = twistY
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    pub = rospyPublisher = rospy.Publisher("/ip_camera_motion", Twist, queue_size=1)
    pub.publish(twist)

def callback(data):
    rospy.loginfo('axes: ' + str(data.axes))
    rospy.loginfo('-----------------')
    if (data.header.seq%3 == 0):
        sendTwist(data.axes[2], data.axes[3])

def listener():
    rospy.init_node('ip_camera_joy', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
