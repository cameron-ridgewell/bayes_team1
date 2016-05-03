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

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <numeric>
#include <functional>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>
//1400 mm away
//x and y variance of find object 2d
static const float CAMERA_SENSOR_X_VAR = 40.9405;
static const float CAMERA_SENSOR_Y_VAR = 921.5028;

//x and y variance of motion of the pan tilt camera
static const float CAMERA_MOTION_X_VAR = 0.0381116;
static const float CAMERA_MOTION_Y_VAR = 0.0486310;

std::string camera_topic = "/networkCam";

static const float FOV_ANGLE = 40; //degrees
static const size_t DETECTION_MAX = 10;

class KalmanProcessor
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber object_sub;
	ros::Subscriber camera_angle_sub;
	cv::Mat src;
	std::vector<float> objects;
	size_t detection_counter;
	Eigen::MatrixXf predicted_pos;
	Eigen::Matrix2f predicted_cov;
	Eigen::MatrixXf estimated_pos;
	Eigen::Matrix2f estimated_cov;
	bool initialized;

	public:
	KalmanProcessor(): it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe(camera_topic, 1, 
			&KalmanProcessor::predict, this);
		object_sub = nh_.subscribe("/centerpoint_detected", 10,
			&KalmanProcessor::update, this);
		camera_angle_sub = nh_.subscribe("/ip_camera_angle", 10,
			&KalmanProcessor::update, this);
		detection_counter = DETECTION_MAX;
		predicted_pos = Eigen::MatrixXf(2,1);
		predicted_cov = Eigen::Matrix2f();
		estimated_pos = Eigen::MatrixXf(2,1);
		estimated_cov = Eigen::Matrix2f();
		initialized = false;
	}

	void predict(const sensor_msgs::ImageConstPtr& msg)
	{
		//only perform predictions if a detection has occured
		if (initialized)
		{
			//Motion Model: F
			Eigen::Matrix2f F = Eigen::Matrix2f();
			F.setIdentity();

			//Sensor Motion Model
			Eigen::Matrix2f B = Eigen::Matrix2f();
			B(0,0) = 
			/*
			 * TODO
			 * Set u_k equal to the cumulative motion of the 
			 * camera since the last update?
			 */
			//Sensor Motion
			Eigen::MatrixXf(2,1) u_k = Eigen::MatrixXf(2,1);

			predicted_pos = F * estimated_pos + B * u_k;
			predicted_cov = F * estimated_cov + Q;
		}
	}

	void update(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		//if its the first detection of a series, set the initial 
		//  position equal to the detected position
		if (!initialized) 
		{
			estimated_pos(0,0) = msg->data[0]; //x position
			estimated_pos(1,0) = msg->data[1]; //y position

			initialized = true;
		}

		//rows, columns
		//Initialize variables
		//Observation: z_k
		Eigen::MatrixXf z_k = Eigen::MatrixXf(2,1);
		z_k(0,0) = msg->data[0]; //x position
		z_k(1,0) = msg->data[1]; //y position

		//Sensor Model: H
		Eigen::Matrix2f H = Eigen::Matrix2f();
		H.setIdentity();
		Eigen::MatrixXf resid = z_k - (H * predicted_pos);
		
	}
};

int main(int argc, char** argv)
{
	//camera_topic = argv[1];
	ros::init(argc, argv, "kalman_processor");
	KalmanProcessor kp;
	ros::spin();
	return 0;
}