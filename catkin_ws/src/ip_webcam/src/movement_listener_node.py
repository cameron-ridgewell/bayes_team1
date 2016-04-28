#!/usr/bin/env python
import sys, tty, termios
import urllib2, rospy
from time import sleep
from geometry_msgs.msg import Twist

def getTwist(twist):

    # up
	return 0;
    # down
	return 2;
    # right
	return 6;
    # left
	return 4;


def sendOneMovement(comm):
    ip = rospy.get_param("camera_ip", "192.168.1.12");
    port = rospy.get_param("camera_port", "81");
    url_prefix = 'http://' + ip + ':' + port + '/decoder_control.cgi?loginuse=admin&loginpas=12345'

    response = urllib2.urlopen(url_prefix + '&command='+str(comm)+'&onestep=0&14434782369140.2543632062152028&_=1443478236914');
    sleep(0.1);
    urllib2.urlopen(url_prefix +'command=7&onestep=1&14434782369140.2543632062152028&_=1443478236914');


def sendContinuousMovement(comm):
    ip = rospy.get_param("camera_ip", "192.168.1.12");
    port = rospy.get_param("camera_port", "81");
    url_prefix = 'http://' + ip + ':' + port + '/decoder_control.cgi?loginuse=admin&loginpas=12345'

    response = urllib2.urlopen(url_prefix + '&command='+str(comm)+'&onestep=0&14434782369140.2543632062152028&_=1443478236914');


def main():
	rospy.init_node('ip_camera_listener', anonymous=True)
    rospy.Subscriber('ip_camera_motion', Twist, callback)

if __name__=='__main__':
        main()
