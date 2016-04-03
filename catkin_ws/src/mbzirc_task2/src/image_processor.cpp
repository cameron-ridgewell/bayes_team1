#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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

//CV_Bridge Variables
static const std::string OPENCV_WINDOW = "Kinect Image";
std::string camera_topic = "/front_kinect/rgb/image";
static const size_t KINECTHEIGHT_RES = 1024;
static const size_t KINECTWIDTH_RES = 1280;


class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:
	ImageConverter(): it_(nh_)
	{
		// Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe(camera_topic, 1, 
		&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_processor/image", 1);
		cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
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

		/*
		 * TODO
		 * Any actual image analysis here
		 */


		cv::imshow(OPENCV_WINDOW, cv_ptr->image);

		/// Wait until user exit program by pressing a key
		cv::waitKey(3);
		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
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