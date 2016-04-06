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

class ImageCropper
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber object_sub;
	ros::Subscriber twist_sub;
	cv::Mat src;
	public:
	ImageCropper(): it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe(camera_topic, 1, 
			&ImageCropper::imageCb, this);
		image_pub_ = it_.advertise("/front_kinect/rgb/image_cropped", 1);
		/* Have to crop the depth data too to use this for find object3d*/
	}

	~ImageCropper()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		//get image cv::Pointer
		cv_bridge::CvImagePtr cv_ptr;// = cv_bridge::toCvShare(msg);

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		src = cv_ptr->image;

		cv::Rect roi(0,0, src.size().width, (2.0/3.0)*src.size().height);

		cv::Mat cropped_image = src(roi);

		cv::waitKey(3);
		// Output modified video stream
		cv_bridge::CvImage out_msg;
		out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
		out_msg.encoding = cv_ptr->encoding; // Or whatever
		out_msg.image    = cropped_image;
		image_pub_.publish(out_msg.toImageMsg());
	}
};

int main(int argc, char** argv)
{
	//camera_topic = argv[1];
	ros::init(argc, argv, "image_cropper");
	ImageCropper ic;
	ros::spin();
	return 0;
}