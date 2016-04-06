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
#include <numeric>
#include <functional>
#include <boost/lexical_cast.hpp>

#include <geometry_msgs/Twist.h>

//CV_Bridge Variables
static const std::string OPENCV_WINDOW = "Kinect Image";
std::string camera_topic = "/front_kinect/rgb/image";
static const size_t KINECTHEIGHT_RES = 1024;
static const size_t KINECTWIDTH_RES = 1280;

float x_pos = -1;
float y_pos = -1;
float depth = -1;

const size_t DETECTION_MAX = 10;

//Formatting variables
static const size_t BOUNDING_BOX_LINE_WIDTH = 1;
static const size_t CROSSHAIR_WIDTH = 10;
static const size_t BELIEF_WIDTH = 1.5;

//Prediction Variables
tf::StampedTransform previous_transform;
static const float HORIZ_FOV_ANGLE = 66.666667;
bool movement = false;
const size_t MOVEMENT_MAX = 20;
size_t movement_counter = 0;

//Kalman Array
float k_array[6];

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
	tf::TransformListener listener;
	cv::Point2f prediction_mean;

	ros::Publisher k_variables;

	public:
	ImageConverter(): it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe(camera_topic, 1, 
			&ImageConverter::imageCb, this);
		object_sub = nh_.subscribe("/objects", 10,
			&ImageConverter::interceptObjects, this);
		image_pub_ = it_.advertise("/image_processor/image", 1);
		k_variables = nh_.advertise<std_msgs::Float32MultiArray>("/kalman_processor", 100);
		cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
		detection_counter = DETECTION_MAX;
		prediction_mean = cv::Point2f(0,0);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		twistPrediction();
		//get image cv::Pointer
		cv_bridge::CvImagePtr cv_ptr;// = cv_bridge::toCvShare(msg);

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		src = cv_ptr->image;

		cv::Rect roi(0,0, src.size().width, (2.0/3.0)*src.size().height);

		cv::Mat cropped_image = src(roi);
		/*
		 * TODO
		 * Any actual image analysis here
		 */
		if (detection_counter >= DETECTION_MAX)
		{
			objects.clear();
			x_pos=-1;
			y_pos=-1;
			depth=-1;
		} else {
			drawPrediction(prediction_mean);
		}
		if (movement_counter >= MOVEMENT_MAX)
		{
			movement = false;
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
		if (objects.size() > 0)
		{
			drawRectFromHomography(object_homs[smallest_element_index(heights)]);
		}
		detection_counter++;
		movement_counter++;

		 
		cv::imshow(OPENCV_WINDOW, src);
		/// Wait until user exit program by pressing a key
		cv::waitKey(3);
		// Output modified video stream
		cv_bridge::CvImage out_msg;
		out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC3; // Or whatever
		out_msg.image    = cropped_image;
		image_pub_.publish(out_msg.toImageMsg());
	}

	void interceptObjects(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		if ((msg->data.size()) > 1)
		{
			objects = msg->data;
			detection_counter = 0;
		}
	}

	void twistPrediction()
	{
		try {
			tf::StampedTransform transform;
			listener.lookupTransform("/base_link", "/odom", ros::Time(0), transform);
			float delta_x = transform.getOrigin().x() - previous_transform.getOrigin().x();
			float delta_y = transform.getOrigin().y() - previous_transform.getOrigin().y();
			float delta_z = transform.getOrigin().z() - previous_transform.getOrigin().z();

			double roll, pitch, yaw, roll2, pitch2, yaw2;
			tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
			tf::Matrix3x3(previous_transform.getRotation()).getRPY(roll2, pitch2, yaw2);
			yaw = yaw * 180 / 3.1415926535;
			yaw2 = yaw2 * 180 / 3.1415926535;
			float delta_yaw = (((yaw - yaw2) < 180) ? (yaw - yaw2) : (yaw2 + 360) - yaw);
			
			if (delta_yaw > 1)
			{
				movement = true;
			}

			k_array[3] = delta_x;
			k_array[4] = delta_y;
			k_array[5] = delta_yaw;
			std_msgs::Float32MultiArray msg;
			std::vector<float> tmp;
			for (int i = 0; i < (sizeof(k_array)/sizeof(*k_array)); i++)
			{
				tmp.push_back(k_array[i]);
			}
			msg.data = tmp;
			k_variables.publish(msg);

			previous_transform = transform;
			prediction_mean.x = prediction_mean.x - delta_yaw / HORIZ_FOV_ANGLE * src.size().width;
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(0.1).sleep();
	    }
	}

	void drawRectFromHomography(const float msg[])
	{
		//    0      1       2     3    4    5    6    7    8    9  10   11   12
		//[obj_id, height, width, m11, m12, m13, m21, m22, m23, m2, m31, m32, m33]
		// Find corners OpenCV
		// get data

	    try{
	    	tf::StampedTransform transform;
	    	listener.lookupTransform("front_camera_viewpoint", "object_" 
				+ boost::lexical_cast<std::string>(msg[0]), ros::Time(0), transform);
			//These are messed up for some reason and have to be reassigned
			float x = transform.getOrigin().x();
			float y = transform.getOrigin().y();
			float z = transform.getOrigin().z();
			k_array[0] = z;
			k_array[1] = x;
			k_array[2] = y;

			//std::cout << "(x,y,z): " << x << "," << y << "," << z << std::endl;
	    }
	    catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(0.1).sleep();
	    }

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
		cv::line(src, outPts.at(0), outPts.at(1), cv::Scalar(0,0,255), BOUNDING_BOX_LINE_WIDTH);
		cv::line(src, outPts.at(1), outPts.at(3), cv::Scalar(0,0,255), BOUNDING_BOX_LINE_WIDTH);
		cv::line(src, outPts.at(3), outPts.at(2), cv::Scalar(0,0,255), BOUNDING_BOX_LINE_WIDTH);
		cv::line(src, outPts.at(2), outPts.at(0), cv::Scalar(0,0,255), BOUNDING_BOX_LINE_WIDTH);

		cv::Point2f sum  = std::accumulate(outPts.begin(), outPts.end(), cv::Point2f(0,0));
		cv::Point2f mean(sum.x / outPts.size(), sum.y / outPts.size());
		x_pos = mean.x;
		y_pos = mean.y;
		drawBeliefSpace(mean, 6);
		if (detection_counter < 1 && !movement)
		{
			prediction_mean = mean;
		}
	}

	void drawBeliefSpace(const cv::Point2f point, const int radius)
	{
		cv::circle(src, point, radius, cv::Scalar(255,0,0), BELIEF_WIDTH);
	}

	void drawPrediction(const cv::Point2f point)
	{
		cv::line(src, point - cv::Point2f(CROSSHAIR_WIDTH, 0), point + cv::Point2f(CROSSHAIR_WIDTH, 0), 
			cv::Scalar(0,0,0), 3);
		cv::line(src, point - cv::Point2f(0, CROSSHAIR_WIDTH), point + cv::Point2f(0, CROSSHAIR_WIDTH),
			cv::Scalar(0,0,0), 3);
	}

	void drawKalmanPrediction(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		cv::Point2f point = cv::Point2f(msg->data[0], msg->data[1]);
		cv::line(src, point - cv::Point2f(CROSSHAIR_WIDTH, 0), point + cv::Point2f(CROSSHAIR_WIDTH, 0), 
			cv::Scalar(0,0,0), 3);
		cv::line(src, point - cv::Point2f(0, CROSSHAIR_WIDTH), point + cv::Point2f(0, CROSSHAIR_WIDTH),
			cv::Scalar(0,0,0), 3);
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