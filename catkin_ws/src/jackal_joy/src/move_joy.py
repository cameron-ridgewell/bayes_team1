#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import urllib2
import time 
import cv2
from geometry_msgs.msg import Twist

def sendTwist(twistX, twistY):
	twist = Twist()
	twist.linear.x = twistX
	twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = twistY
	pub = rospyPublisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub.publish(twist)
	
def callback(data):
  
	rospy.loginfo('axes: ' + str(data.axes))
	rospy.loginfo('-----------------')

	sendTwist(data.axes[1],data.axes[0])
    
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('jackal_joy', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
