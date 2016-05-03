#!/usr/bin/env python

import rospy, cv2, urllib2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def main():
	
	pub = rospyPublisher = rospy.Publisher("/networkCam/rospy", Image,
		queue_size=1, buff_size=2**24)

    ip = rospy.get_param("camera_ip", "192.168.1.12");
    port = rospy.get_param("camera_port", "81");

	url = "http://" + ip + ":" + port + "/videostream.cgi?loginuse=admin&loginpas=12345&dummy=param.mjpg";

    while not rospy.is_shutdown():
    	response = urllib2.urlopen(urllib2.Request(url))
    	img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
    	frame = cv2.imdencode(img_array,1)
    	pub.publish(CvBridge().cv2_to_imgmsg(frame, "bgr8"))

if __name__=='__main__':
	rospy.init_node('ip_camera_image', anonymous=True)
    main()
