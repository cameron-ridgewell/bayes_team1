#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <opencv2/opencv.hpp>

struct cameraObject{
	std::string ip;
	std::string port;
	std::string streamURL;
	std::string commands;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher imagePublisher = it.advertise("/networkCam", 1);

	std::string ip, port; 
	nh.param<std::string>("camera_ip", ip, "192.168.1.12");
	nh.param<std::string>("camera_port", port, "81");
	
	cameraObject networkCamera;
	networkCamera.ip = "http://" + ip;
	networkCamera.port = port;
	networkCamera.streamURL = "/videostream.cgi?loginuse=admin&loginpas=12345&dummy=param.mjpg";

	std::string fullURL = networkCamera.ip + ":" + networkCamera.port + networkCamera.streamURL;
	std::cout<<"URL to stream from: "<< fullURL << std::endl;
	cv::VideoCapture capture(fullURL);
	std::cout<<"capture object initialized" << std::endl;

	cv::Mat frame;
	cv_bridge::CvImage rosImage;

	rosImage.encoding = "bgr8";
	rosImage.header.stamp = ros::Time::now();
	rosImage.header.frame_id = "bar";
	while(nh.ok())
	{
		capture >> frame;
		rosImage.image = frame.clone();

		imagePublisher.publish(rosImage.toImageMsg());
		ros::spinOnce();
	}

  return 0;
}

