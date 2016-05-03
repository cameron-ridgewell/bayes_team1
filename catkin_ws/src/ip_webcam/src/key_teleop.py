#!/usr/bin/env python
import sys,tty,termios
import urllib2, rospy
from time import sleep
from geometry_msgs.msg import Twist

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

def get():
	inkey = _Getch()
	while(exit):
		k=inkey()
		if k!='':break
	if k=='\x1b[A':
		print "up"
		return 0;
	elif k=='\x1b[B':
		print "down"
		return 2;
	elif k=='\x1b[C':
		print "right"
		return 6;
	elif k=='\x1b[D':
		print "left"
		return 4;
	else:
		print "press key again to exit"
		return -1;

def sendCommand(comm):
	twist = Twist()
	if (comm == 0):
		twist.linear.y = 1
	elif (comm == 2):
		twist.linear.y = -1
	elif (comm == 4):
		twist.linear.x = 1
	elif (comm == 6):
		twist.linear.x = -1
	if (comm >= 0):
		pub = rospyPublisher = rospy.Publisher("/ip_camera_motion", Twist, queue_size=10)
		pub.publish(twist)

def main():
	rospy.init_node('key_teleop', anonymous=True)
	last = get();
        while(last >= 0):
			last = get()
			if last >= 0:
				sendCommand(last)

if __name__=='__main__':
        main()
