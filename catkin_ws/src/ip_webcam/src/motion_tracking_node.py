#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import urllib2
import time 

def control_motors(error_x, error_y):
	if(error_x>15):
		urlExecution(4)
	else:
		urlExecution(6)
	if(error_y>15):
		urlExecution(0)
	else:
		urlExecution(2)

def urlExecution(command):
	ip = 'http://192.168.1.41:81/decoder_control.cgi?loginuse=admin&loginpas=12345&command='
	oneStep = '&onestep=1&'
	gibberish = '7485621407675288&_='
	timeStamp = int(time.time())*1000
	fullURL = ip+str(command)+oneStep+str(timeStamp)+'.49641236611690986&_='+str(timeStamp)
	response = urllib2.urlopen(fullURL)
	time.sleep(0.2)
	timeStamp = int(time.time())*1000
	urllib2.urlopen(ip+"7"+'onestep=0&'+ str(timeStamp)+'.49641236611690986&_='+str(timeStamp))
	rospy.loginfo(fullURL)

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
