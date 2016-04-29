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
