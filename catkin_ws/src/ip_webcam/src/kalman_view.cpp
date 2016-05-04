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

static const size_t DETECTION_MAX = 10;
static const std::string CAMERA_TOPIC = "/networkCam";
class KalmanView
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber object_sub;
	ros::Subscriber prediction;
	ros::Subscriber estimation;
	size_t detection_counter;
	Eigen::MatrixXf detected_centerpoint;
	Eigen::MatrixXf predicted_pos;
	Eigen::Matrix2f predicted_cov;
	Eigen::MatrixXf estimated_pos;
	Eigen::Matrix2f estimated_cov;
	cv::Mat src;

	public:
	KalmanView(): it_(nh_)
	{
		image_sub_ = it_.subscribe(CAMERA_TOPIC, 1, 
			&KalmanView::showImage, this);
		object_sub = nh_.subscribe("/centerpoint_detected", 1,
			&KalmanView::updateCenterpoint, this);
		prediction = nh_.subscribe("/kalman/prediction", 1, 
			&KalmanView::updatePrediction, this);
		estimation = nh_.subscribe("/kalman/estimation", 1, 
			&KalmanView::updateEstimation, this);
		image_pub_ = it_.advertise("/kalman_viewer/image", 1);

		detection_counter = DETECTION_MAX;

		detected_centerpoint = Eigen::MatrixXf(2,1);
		predicted_pos = Eigen::MatrixXf(2,1);
		predicted_cov = Eigen::Matrix2f();
		estimated_pos = Eigen::MatrixXf(2,1);
		estimated_cov = Eigen::Matrix2f();
	}

	void updateCenterpoint(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		detected_centerpoint(0,0) = msg->data[0]; //x position
		detected_centerpoint(1,0) = msg->data[1]; //y position
	}

	void updatePrediction(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		predicted_pos(0,0) = msg->data[0]; //x position
		predicted_pos(1,0) = msg->data[1]; //y position
		predicted_cov(0,0) = msg->data[2];
		predicted_cov(0,1) = msg->data[3];
		predicted_cov(1,0) = msg->data[4];
		predicted_cov(1,1) = msg->data[5];
	}

	void updateEstimation(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		estimated_pos(0,0) = msg->data[0]; //x position
		estimated_pos(1,0) = msg->data[1]; //y position
		estimated_cov(0,0) = msg->data[2];
		estimated_cov(0,1) = msg->data[3];
		estimated_cov(1,0) = msg->data[4];
		estimated_cov(1,1) = msg->data[5];
	}

	void showImage(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;// = cv_bridge::toCvShare(msg);
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		src = cv_ptr->image;

		drawPrediction();
		drawEstimation();

		// Output modified video stream
		cv_bridge::CvImage out_msg;
		out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
		out_msg.image    = src;
		image_pub_.publish(out_msg.toImageMsg());
	}

	void drawPrediction()
	{
		if (predicted_pos(0,0) > 0 && predicted_pos(1,0) > 0)
		{
			// cv::circle(src, cv::Point2f(predicted_pos(0,0), predicted_pos(1,0)), 
			// 	3, cv::Scalar(255,0,0), 2);
			cv::ellipse(src, cv::Point2f(predicted_pos(0,0), predicted_pos(1,0)),
				cv::Size(predicted_cov(1,1)*2, predicted_cov(0,0)*2), 
				0.0, 0.0, 360.0, cv::Scalar(255,0,0), 1);
		}
	}

	void drawEstimation()
	{
		if (estimated_pos(0,0) > 0 && estimated_pos(1,0) > 0)
		{
			// cv::circle(src, cv::Point2f(predicted_pos(0,0), predicted_pos(1,0)), 
			// 	3, cv::Scalar(255,0,0), 2);
			cv::ellipse(src, cv::Point2f(estimated_pos(0,0), estimated_pos(1,0)),
				cv::Size(estimated_cov(1,1)*2, estimated_cov(0,0)*2), 
				0.0, 0.0, 360.0, cv::Scalar(0,255,0), 1);
		}
	}
};

int main(int argc, char** argv)
{
	//CAMERA_TOPIC = argv[1];
	ros::init(argc, argv, "kalman_viewer");
	KalmanView kp;
	ros::spin();
	return 0;
}