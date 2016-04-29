#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import urllib2
import time 
from geometry_msgs.msg import Twist

def callback(data):
	rospy.loginfo(type(data.data))
	rospy.loginfo(data.data)
	if(len(data.data)>0):
		rospy.loginfo('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
		rospy.loginfo('X: ' + str(data.data[9]))
		rospy.loginfo('Y: ' + str(data.data[10]))
		error_x= 320.0-data.data[9];
		error_y= 240-data.data[10];
		control_motors(error_x,error_y);

    
def listener():
    rospy.init_node('object_tracking', anonymous=True)

    rospy.Subscriber("/objects", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
