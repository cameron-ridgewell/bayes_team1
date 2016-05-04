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

class OdomManager
{
	ros::NodeHandle nh_;
	tf::TransformListener listener;
	ros::Subscriber tf_sub;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Publisher odom_pos;

	public:
	OdomManager(): it_(nh_)
	{
		image_sub_ = it_.subscribe("/networkCam", 1, 
			&OdomManager::getOdom, this);
		odom_pos = nh_.advertise<std_msgs::Float32MultiArray>("/odom/position", 1);
	}

	void getOdom(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			tf::StampedTransform transform;
			listener.lookupTransform("/base_link", "/odom", ros::Time(0), transform);
			
			double roll, pitch, yaw;
			tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
			roll = roll * 180 / 3.1415926535;
			pitch = pitch * 180 / 3.1415926535;
			yaw = yaw * 180 / 3.1415926535;
			
			std::vector<float> v;
			v.push_back(transform.getOrigin().x());
			v.push_back(transform.getOrigin().y());
			v.push_back(transform.getOrigin().z());
			v.push_back(roll);
			v.push_back(pitch);
			v.push_back(yaw);
			std_msgs::Float32MultiArray msg;
			msg.data = v;
			odom_pos.publish(msg);
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(0.1).sleep();
	    }

	}
};

int main(int argc, char** argv)
{
	//CAMERA_TOPIC = argv[1];
	ros::init(argc, argv, "odom_manager");
	OdomManager om;
	ros::spin();
	return 0;
}