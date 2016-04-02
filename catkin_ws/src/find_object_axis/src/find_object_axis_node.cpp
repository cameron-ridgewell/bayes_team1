#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;
using namespace std;

Mat image;
int objRan = 0;
int imgRan = 0;

void displayImage(std::vector<cv::Point2f> outPts)
{
	if (!image.empty()) {
    		if (objRan + imgRan == 2)
		{
			//Find center point and axes
			float x0 = (outPts.at(0).x + outPts.at(1).x + outPts.at(2).x + outPts.at(3).x) / 4.0;
			float y0 = (outPts.at(0).y + outPts.at(1).y + outPts.at(2).y + outPts.at(3).y) / 4.0;
			float xi = (outPts.at(1).x + outPts.at(3).x) / 2.0;
			float yi = (outPts.at(1).y + outPts.at(3).y) / 2.0;
			float xj = (outPts.at(2).x + outPts.at(3).x) / 2.0;
			float yj = (outPts.at(2).y + outPts.at(3).y) / 2.0;

			float xk = ((outPts.at(2).x + outPts.at(3).x) / 2.0) - ((outPts.at(0).x + outPts.at(1).x) / 2.0) + x0;
			float yk = ((outPts.at(1).y + outPts.at(3).y) / 2.0) - ((outPts.at(0).y + outPts.at(2).y) / 2.0) + y0;

			float width = (outPts.at(1).x - outPts.at(0).x - outPts.at(2).x + outPts.at(3).x) / 2.0;
			float height = (outPts.at(2).y - outPts.at(0).y - outPts.at(1).y + outPts.at(3).y) / 2.0;
			cout<<height<<std::endl;

	        	//Draw a rectangle and lines
			//Convex Poly params: (image, points, number of points, RGB color)
	        	//Line params: (image, point 1, point 2, RGB color, line thickness)
	        	//A thickness of -1 tells it to fill in the shape
	        	//Colors are in BGR order instead of RGB for some reason
			Point* pts = new Point[4];
			pts[0] = Point(outPts.at(0).x + (width * 0.3), outPts.at(0).y + (height * 0.2));
			pts[1] = Point(outPts.at(1).x - (width * 0.3), outPts.at(1).y + (height * 0.2));
			pts[3] = Point(outPts.at(2).x + (width * 0.3), outPts.at(2).y - (height * 0.2));
			pts[2] = Point(outPts.at(3).x - (width * 0.3), outPts.at(3).y - (height * 0.2));
			if (height > 290)
			{
				fillConvexPoly(image, pts, 4, Scalar(0, 255, 0));
			}
			else
			{
				fillConvexPoly(image, pts, 4, Scalar(0, 0, 255));
			}
	        	line(image, Point(x0, y0), Point(xi, yi), Scalar(255, 0, 0), 2);
	        	line(image, Point(x0, y0), Point(xj, yj), Scalar(255, 255, 0), 2);
			line(image, Point(x0, y0), Point(xk, yk), Scalar(0, 255, 255), 2);

			imshow("Display window", image);                   // Show our image inside it.
		} 
	}
   
}

void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
	if(msg.data.size())
	{
		objRan = 1;
		for(unsigned int i=0; i<msg.data.size(); i+=12)
		{
			// get data
			int id = (int)msg.data[i];
			float objectWidth = msg.data[i+1];
			float objectHeight = msg.data[i+2];

			// Example with OpenCV
			// Find corners OpenCV
			cv::Mat cvHomography(3, 3, CV_32F);
			cvHomography.at<float>(0,0) = msg.data[i+3];
			cvHomography.at<float>(1,0) = msg.data[i+4];
			cvHomography.at<float>(2,0) = msg.data[i+5];
			cvHomography.at<float>(0,1) = msg.data[i+6];
			cvHomography.at<float>(1,1) = msg.data[i+7];
			cvHomography.at<float>(2,1) = msg.data[i+8];
			cvHomography.at<float>(0,2) = msg.data[i+9];
			cvHomography.at<float>(1,2) = msg.data[i+10];
			cvHomography.at<float>(2,2) = msg.data[i+11];
			std::vector<cv::Point2f> inPts, outPts;
			inPts.push_back(cv::Point2f(0,0));
			inPts.push_back(cv::Point2f(objectWidth,0));
			inPts.push_back(cv::Point2f(0,objectHeight));
			inPts.push_back(cv::Point2f(objectWidth,objectHeight));
			cv::perspectiveTransform(inPts, outPts, cvHomography);

			printf("Object %d detected, CV corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					id,
					outPts.at(0).x, outPts.at(0).y,
					outPts.at(1).x, outPts.at(1).y,
					outPts.at(2).x, outPts.at(2).y,
					outPts.at(3).x, outPts.at(3).y);
			displayImage(outPts);
		}
	}
	else
	{
		printf("No objects detected.\n");
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		imgRan = 1;
		image = cv_bridge::toCvShare(msg, "bgr8")->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main( int argc, char** argv )
{
	namedWindow("Display window", WINDOW_AUTOSIZE);// Create a window for display.
	ros::init(argc, argv, "objects_detected");
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/front_kinect/rgb/image", 1, imageCallback);

	ros::NodeHandle nh2;
	ros::Subscriber subs;
	subs = nh2.subscribe("/objects", 1, objectsDetectedCallback);

	ros::spin();
	return 0;
}
