#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
std::string subscribed_cloud; 

void callback(const PointCloud::ConstPtr& sub_msg)
{
 	ros::NodeHandle nh;
	ros::Publisher pub = 
		nh.advertise<PointCloud>(subscribed_cloud + "_transformed", 1);

	pcl::PointCloud<pcl::PointXYZRGB> pc;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0, 0, 0) );
	tf::Quaternion q;
  	q.setRPY(3.1415926535897/2, 3.1415926535897, 3.1415926535897/2);
  	transform.setRotation(q);
	pcl_ros::transformPointCloud(*sub_msg, pc, transform);

	if (nh.ok())
	{
		pub.publish (pc);
		ros::spinOnce ();
	}
}

int main(int argc, char** argv)
{
	subscribed_cloud = argv[1];
  	ros::init(argc, argv, "pcl_transform");
  	ros::NodeHandle nh;
  	ros::Subscriber sub = nh.subscribe<PointCloud>(subscribed_cloud, 1, callback);
  	ros::spin();
}