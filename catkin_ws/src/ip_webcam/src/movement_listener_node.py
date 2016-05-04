#!/usr/bin/env python
import sys, tty, termios
import urllib2, rospy
from time import sleep
from geometry_msgs.msg import Twist

x_average_degs = 1.3178917
y_average_degs = 1.3267467

x_deg_pos = 0;
y_deg_pos = 0;

def getTwist(twist):
    global x_deg_pos
    global y_deg_pos
    THRES = 0.85
    x = twist.linear.x
    y = twist.linear.y
    if (x < THRES and x > -THRES and y > THRES):
        # up
        y_deg_pos += y_average_degs
        return 0;
    elif (x < THRES and x > -THRES and y < -THRES):
        # down
        y_deg_pos -= y_average_degs
        return 2;
    elif (x > THRES and y > -THRES and y < THRES):
        # right
        x_deg_pos += x_average_degs
        return 4;
    elif (x < -THRES and y > -THRES and y < THRES):
        # left
        x_deg_pos -= x_average_degs
        return 6;
    elif (x < -THRES and y < -THRES):
        # down-left
        y_deg_pos -= y_average_degs
        x_deg_pos += x_average_degs
        return 93;
    elif (x > THRES and y < -THRES):
        #down-right
        y_deg_pos -= y_average_degs
        x_deg_pos += x_average_degs
        return 92;
    elif (x < -THRES and y > THRES):
        #up-left
        y_deg_pos += y_average_degs
        x_deg_pos -= x_average_degs
        return 91;
    elif (x > THRES and y > THRES):
        #up-right
        y_deg_pos += y_average_degs
        x_deg_pos += x_average_degs
        return 90;
    else:
        return -1;

def sendOneMovement(comm):
    if comm < 0:
        return;
    ip = rospy.get_param("camera_ip", "192.168.1.12");
    port = rospy.get_param("camera_port", "81");
    url_prefix = 'http://' + ip + ':' + port + '/decoder_control.cgi?loginuse=admin&loginpas=12345'

    response = urllib2.urlopen(url_prefix + '&command='+str(comm)+'&onestep=0&14434782369140.2543632062152028&_=1443478236914');
    sleep(0.1);
    urllib2.urlopen(url_prefix +'&command=7&onestep=1&14434782369140.2543632062152028&_=1443478236914');
    pub = rospyPublisher = rospy.Publisher("/ip_camera_angle", Twist, queue_size=10)
    twist = Twist()
    global x_deg_pos
    global y_deg_pos
    twist.angular.z = x_deg_pos
    twist.angular.x = y_deg_pos
    pub.publish(twist)

def getTwistHandler(twist):
    val = getTwist(twist)
    if val != -1:
        print val
    sendOneMovement(val)

def main():
    rospy.init_node('ip_camera_listener', anonymous=True)
    rospy.Subscriber('/ip_camera_motion', Twist, getTwistHandler)
    rospy.spin()

if __name__=='__main__':
        main()
