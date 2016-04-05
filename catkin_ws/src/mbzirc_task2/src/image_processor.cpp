#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Float32MultiArray.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>

//CV_Bridge Variables
static const std::string OPENCV_WINDOW = "Kinect Image";
std::string camera_topic = "/front_kinect/rgb/image";
static const size_t KINECTHEIGHT_RES = 1024;
static const size_t KINECTWIDTH_RES = 1280;

float x_pos = 0;
float y_pos = 0;
float width = 0;
float height = 0;

const size_t DETECTION_MAX = 10;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber object_sub;
	cv::Mat src;
	std::vector<float> objects;
	size_t detection_counter;

	public:
	ImageConverter(): it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe(camera_topic, 1, 
			&ImageConverter::imageCb, this);
		object_sub = nh_.subscribe("/objects", 10,
			&ImageConverter::interceptObjects, this);
		image_pub_ = it_.advertise("/image_processor/image", 1);
		cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
		detection_counter = DETECTION_MAX;
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//get image cv::Pointer
		cv_bridge::CvImagePtr cv_ptr;// = cv_bridge::toCvShare(msg);

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		src = cv_ptr->image;
		/*
		 * TODO
		 * Any actual image analysis here
		 */
		std::cout << "det: " << detection_counter << "\n";
		if (detection_counter >= DETECTION_MAX)
		{
			objects.clear();
		}

		std::vector<float> heights;
		std::vector<float*> object_homs;
		for (int i = 0; i < objects.size(); i+=12)
		{
			float arr[12];
			for (int j = 0; j < 12; j++)
			{
				arr[j] = objects[j+i];
			}
			//Only draw top object detected
			heights.push_back(arr[1]);
			object_homs.push_back(arr);
		}
		std::cout << "size: " << objects.size() << "\n"; 
		if (objects.size() > 0)
		{
			drawRectFromHomography(object_homs[smallest_element_index(heights)]);
		}
		detection_counter++;

		 
		cv::imshow(OPENCV_WINDOW, src);
		/// Wait until user exit program by pressing a key
		cv::waitKey(3);
		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}

	void interceptObjects(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		if ((msg->data.size()) > 1)
		{
			objects = msg->data;
			detection_counter = 0;
		}
	}

	void drawRectFromHomography(const float msg[])
	{
		//    0      1       2     3    4    5    6    7    8    9  10   11   12
		//[obj_id, height, width, m11, m12, m13, m21, m22, m23, m2, m31, m32, m33]
		// Find corners OpenCV
		// get data
		size_t i=0;
		int id = (int) msg[i];
		float objectWidth = msg[i+1];
		float objectHeight = msg[i+2];
		cv::Mat cvHomography(3, 3, CV_32F);
		cvHomography.at<float>(0,0) = msg[i+3];
		cvHomography.at<float>(1,0) = msg[i+4];
		cvHomography.at<float>(2,0) = msg[i+5];
		cvHomography.at<float>(0,1) = msg[i+6];
		cvHomography.at<float>(1,1) = msg[i+7];
		cvHomography.at<float>(2,1) = msg[i+8];
		cvHomography.at<float>(0,2) = msg[i+9];
		cvHomography.at<float>(1,2) = msg[i+10];
		cvHomography.at<float>(2,2) = msg[i+11];
		std::vector<cv::Point2f> inPts, outPts;
		inPts.push_back(cv::Point2f(0,0));
		inPts.push_back(cv::Point2f(objectWidth,0));
		inPts.push_back(cv::Point2f(0,objectHeight));
		inPts.push_back(cv::Point2f(objectWidth,objectHeight));
		cv::perspectiveTransform(inPts, outPts, cvHomography);
		/*
		//Print the detected objects
		printf("Object %d detected, CV corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					id,
					outPts.at(0).x, outPts.at(0).y,
					outPts.at(1).x, outPts.at(1).y,
					outPts.at(2).x, outPts.at(2).y,
					outPts.at(3).x, outPts.at(3).y);
		*/
		cv::line(src, outPts.at(0), outPts.at(1), cv::Scalar(0,0,255));
		cv::line(src, outPts.at(1), outPts.at(3), cv::Scalar(0,0,255));
		cv::line(src, outPts.at(3), outPts.at(2), cv::Scalar(0,0,255));
		cv::line(src, outPts.at(2), outPts.at(0), cv::Scalar(0,0,255));
		
	}

	size_t smallest_element_index(std::vector<float> &array)
	{
		size_t min = array.at(0);
		size_t index = 0;
		for (int i = 1; i < array.size(); i++)
		{
			if (array.at(i) < min)
			{
				index = i;
			}
		}
		return index;
	}
};

int main(int argc, char** argv)
{
	//camera_topic = argv[1];
	ros::init(argc, argv, "image_processor");
	ImageConverter ic;
	ros::spin();
	return 0;
}