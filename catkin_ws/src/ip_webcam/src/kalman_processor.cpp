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
#include <time.h>

//1400 mm away
//x and y variance of find object 2d
static const float CAMERA_SENSOR_X_VAR = 6.9405;
static const float CAMERA_SENSOR_Y_VAR = 30.5028;

//x and y variance of motion of the pan tilt camera
static const float CAMERA_MOTION_X_VAR = .4728;//0.0381116;
static const float CAMERA_MOTION_Y_VAR = .4728;//0.0486310;

static const float IMAGE_WIDTH = 640;
static const float IMAGE_HEIGHT = 480;

static const std::string CAMERA_TOPIC = "/networkCam";

static const float FOV_ANGLE = 40; //degrees
static const time_t DETECTION_MAX = 0;

static bool searching = true;

class KalmanProcessor
{
	ros::NodeHandle nh_;
	ros::Subscriber object_sub;
	ros::Subscriber camera_angle_sub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher prediction;
	ros::Publisher estimation;
	std::vector<float> objects;
	time_t detection_counter;
	Eigen::MatrixXf predicted_pos;
	Eigen::Matrix2f predicted_cov;
	Eigen::MatrixXf estimated_pos;
	Eigen::Matrix2f estimated_cov;
	float last_used_camera_pos[2];
	float last_read_camera_pos[2];
	bool initialized;

	public:
	KalmanProcessor():it_(nh_)
	{
		image_sub_ = it_.subscribe(CAMERA_TOPIC, 1, 
			&KalmanProcessor::checkActive, this);
		object_sub = nh_.subscribe("/centerpoint_detected", 1,
			&KalmanProcessor::update, this);
		camera_angle_sub = nh_.subscribe("/ip_camera_angle", 1,
			&KalmanProcessor::update_camera_angle, this);
		prediction = nh_.advertise<std_msgs::Float32MultiArray>
			("/kalman/prediction",1);
		estimation = nh_.advertise<std_msgs::Float32MultiArray>
			("/kalman/estimation",1);
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

	~KalmanProcessor()
	{
		searching = false;
	}

	void predict()
	{
		//only perform predictions if a detection has occured
		if (initialized)
		{
			//Motion Model: F
			Eigen::Matrix2f F = Eigen::Matrix2f();
			F.setIdentity();

			//Sensor Motion Model
			Eigen::Matrix2f B = Eigen::Matrix2f();
			B(0,0) = IMAGE_WIDTH / FOV_ANGLE;
			B(1,1) = IMAGE_HEIGHT / FOV_ANGLE;

			//Sensor Motion
			Eigen::MatrixXf u_k = Eigen::MatrixXf(2,1);
			u_k(0,0) = last_read_camera_pos[0] 
				- last_used_camera_pos[0];	//camera yaw change since last predicition 
			u_k(1,0) = last_read_camera_pos[1] 
				- last_used_camera_pos[1];	//camera pitch change since last prediction
			std::cout << "camera: " << u_k(0,0) << " " << u_k(1,0) << std::endl;
			last_used_camera_pos[0] = last_read_camera_pos[0];
			last_used_camera_pos[1] = last_read_camera_pos[1];

			//Process Noise Covariance
			Eigen::Matrix2f Q = Eigen::Matrix2f();
			Q(0,0) = CAMERA_MOTION_X_VAR;
			Q(1,1) = CAMERA_MOTION_Y_VAR;
			
			Eigen::MatrixXf tmp = F * estimated_pos + B * u_k;
			if (tmp(0,0) <= IMAGE_WIDTH && tmp(0,0) >= 0 
				&& tmp(1,0) <= IMAGE_HEIGHT && tmp(1,0) >= 0)
			{
				estimated_pos = predicted_pos = F * estimated_pos + B * u_k;
				estimated_cov = predicted_cov = F * estimated_cov + Q;
				std::vector<float> tmp2;
				tmp2.push_back(predicted_pos(0,0));
				tmp2.push_back(predicted_pos(1,0));
				tmp2.push_back(predicted_cov(0,0));
				tmp2.push_back(predicted_cov(0,1));
				tmp2.push_back(predicted_cov(1,0));
				tmp2.push_back(predicted_cov(1,1));
				std_msgs::Float32MultiArray msg;
				msg.data = tmp2;
				prediction.publish(msg);
			}
			if (predicted_pos(0,0) < 1 && predicted_pos(1,0) < 1)
			{
				reset();	
			}
		}
	}

	void update(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		time(&detection_counter);
		//if its the first detection of a series, set the initial 
		//  position equal to the detected position
		if (!initialized) 
		{
			std::cerr << "Initializing...\n";
			estimated_pos(0,0) = msg->data[0]; //x position
			estimated_pos(1,0) = msg->data[1]; //y position
			initialized = true;
			predict();
		}
		else
		{
			predict();
			// predicted_pos(0,0) = msg->data[0]; //x position
			// predicted_pos(1,0) = msg->data[1]; //y position
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

			Eigen::MatrixXf tmp = (predicted_pos + K * resid);
			if (tmp(0,0) <= IMAGE_WIDTH && tmp(0,0) >= 0 
				&& tmp(1,0) <= IMAGE_HEIGHT && tmp(1,0) >= 0)
			{
				//Estimated position
				estimated_pos = predicted_pos + K * resid;

				//Estimated covariance
				estimated_cov = (Eigen::Matrix2f().setIdentity() - K * H) * predicted_cov;

				std::vector<float> tmp2;
				tmp2.push_back(estimated_pos(0,0));
				tmp2.push_back(estimated_pos(1,0));
				tmp2.push_back(estimated_cov(0,0));
				tmp2.push_back(estimated_cov(0,1));
				tmp2.push_back(estimated_cov(1,0));
				tmp2.push_back(estimated_cov(1,1));
				std_msgs::Float32MultiArray msg;
				msg.data = tmp2;
				estimation.publish(msg);
			}
			
			std::cout << estimated_pos << "\nendUpdate\n";
		}
	}

	void update_camera_angle(const geometry_msgs::Twist& twist)
	{
		last_read_camera_pos[0] = twist.angular.z; //yaw
		last_read_camera_pos[1] = twist.angular.x; //pitch
		predict();
	}

	void reset()
	{
		predicted_pos(0,0) = 0;
		predicted_pos(1,0) = 0;
		initialized = false;
	}

	void checkActive(const sensor_msgs::ImageConstPtr& msg)
	{
		time_t now;
		time(&now);
		if(difftime(now, detection_counter) > 4)
		{
			reset();
			std::vector<float> tmp2;
			tmp2.push_back(0);
			tmp2.push_back(0);
			tmp2.push_back(0);
			tmp2.push_back(0);
			tmp2.push_back(0);
			tmp2.push_back(0);
			std_msgs::Float32MultiArray msg;
			msg.data = tmp2;
			prediction.publish(msg);
		} 
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