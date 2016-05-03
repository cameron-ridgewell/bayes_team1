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
	float last_used_camera_pos[2];
	float last_read_camera_pos[2];
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
			&KalmanProcessor::update_camera_angle, this);
		image_pub_ = it_.advertise("/kalman_processor/image", 1);
		detection_counter = DETECTION_MAX;
		predicted_pos = Eigen::MatrixXf(2,1);
		predicted_cov = Eigen::Matrix2f();
		estimated_pos = Eigen::MatrixXf(2,1);
		estimated_cov = Eigen::Matrix2f();
		last_used_camera_pos[0] = 0;
		last_used_camera_pos[1] = 0;
		last_read_camera_pos[0] = 0;
		last_read_camera_pos[1] = 0;
		initialized = false;
	}

	void predict(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;// = cv_bridge::toCvShare(msg);
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		src = cv_ptr->image;

		//only perform predictions if a detection has occured
		if (initialized)
		{
			//Motion Model: F
			Eigen::Matrix2f F = Eigen::Matrix2f();
			F.setIdentity();

			//Sensor Motion Model
			Eigen::Matrix2f B = Eigen::Matrix2f();
			B(0,0) = src.cols / FOV_ANGLE;
			B(1,1) = src.rows / FOV_ANGLE;

			//Sensor Motion
			Eigen::MatrixXf u_k = Eigen::MatrixXf(2,1);
			u_k(0,0) = last_read_camera_pos[0] - last_used_camera_pos[0];	//camera yaw change since last predicition 
			u_k(1,0) = last_read_camera_pos[1] - last_used_camera_pos[1];	//camera pitch change since last prection
			std::cout << u_k(0,0) << " " << u_k(1,0) << std::endl;
			//Process Noise Covariance
			Eigen::Matrix2f Q = Eigen::Matrix2f();
			Q(0,0) = CAMERA_MOTION_X_VAR;
			Q(1,1) = CAMERA_MOTION_Y_VAR;

			predicted_pos = F * estimated_pos + B * u_k;
			predicted_cov = F * estimated_cov + Q;
			drawCircle(cv::Point2f(predicted_pos(0,0), predicted_pos(1,0)), 3, 2);
		}
		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src).toImageMsg();
		image_pub_.publish(img_msg);
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
		else
		{
			//rows, columns
			//Initialize variables
			//Observation: z_k
			Eigen::MatrixXf z_k = Eigen::MatrixXf(2,1);
			z_k(0,0) = msg->data[0]; //x position
			z_k(1,0) = msg->data[1]; //y position

			//Sensor Model: H
			Eigen::Matrix2f H = Eigen::Matrix2f();
			H.setIdentity();
			
			//Residual
			Eigen::MatrixXf resid = z_k - (H * predicted_pos);

			//Sensor Noise Covariance
			Eigen::Matrix2f R = Eigen::Matrix2f();
			R(0,0) = CAMERA_SENSOR_X_VAR;
			R(1,1) = CAMERA_SENSOR_Y_VAR;

			//Residual covariance
			Eigen::Matrix2f resid_cov = H * predicted_cov * H.inverse() + R;

			//Kalman Coefficient
			Eigen::Matrix2f K = predicted_cov * H.inverse() * resid_cov.inverse();

			//Estimated position
			estimated_pos = predicted_pos + K * resid;

			//Estimated covariance
			estimated_cov = (Eigen::Matrix2f().setIdentity() - K * H) * predicted_cov;
			
			std::cout << estimated_pos << "\nendUpdate\n";
		}
	}

	void update_camera_angle(const geometry_msgs::Twist& twist)
	{
		last_read_camera_pos[0] = twist.angular.z; //yaw
		last_read_camera_pos[1] = twist.angular.x; //pitch
	}

	void drawCircle(const cv::Point2f point, const int radius, const size_t circle_width)
	{
		cv::circle(src, point, radius, cv::Scalar(255,0,0), circle_width);
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