#include <ros/ros.h>
#include <dirent.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_processor");
	
	ros::NodeHandle node;

	tf::TransformListener listener;
	static tf::TransformBroadcaster br;

	std::string mbzirc_path = ros::package::getPath("mbzirc_task2");
	std::string wrench_objects_path = mbzirc_path + "//wrench_objects//";

  	DIR *dp;
  	dirent *d;
  	std::vector<std::string> object_names;
  	if((dp = opendir(wrench_objects_path.c_str())) != NULL) 
  	{
  		std::cout << "Searching for objects: ";
  		while ((d = readdir(dp)) != NULL)
  		{
  			if(!strcmp(d->d_name,".") || !strcmp(d->d_name,".."))
				continue;
			std::string arr[2] = std::strtok(d->d_name, ".");
			object_names.push_back("object_" + arr[0]);
			std::cout << d->d_name << " ";

  		}
  		std::cout << "\n" << "All Objects:\n";
		for (std::vector<std::string>::const_iterator i = object_names.begin(); i != object_names.end(); ++i)
		    std::cout << *i << ' ';
  	}


	ros::Rate rate(10.0);
  	while (node.ok())
  	{
  		tf::StampedTransform transform;
  		tf::StampedTransform newtransform;
  		for (std::vector<std::string>::const_iterator i = object_names.begin(); i != object_names.end(); ++i)
  		{
		    try{
		      	listener.lookupTransform("front_camera_viewpoint", *i, 
		      		ros::Time(0), transform);
		    	newtransform.setOrigin(tf::Vector3(transform.getOrigin().z(),
		    		-transform.getOrigin().x(),
		    		-transform.getOrigin().y()));
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				/*
				 * TODO
				 * Make the correct import of pose
				 */
				newtransform.setRotation(q);
		    	br.sendTransform(tf::StampedTransform(newtransform, ros::Time::now(), "front_camera_viewpoint", "wrench"));
		    	
		    	listener.lookupTransform("odom", *i, 
		      		ros::Time(0), transform);
		    	newtransform.setOrigin(tf::Vector3(transform.getOrigin().z(),
		    		-transform.getOrigin().x(),
		    		-transform.getOrigin().y()));
				tf::Quaternion q;
				q.setRPY(0, 0, 0);
				/*
				 * TODO
				 * Make the correct import of pose
				 */
				newtransform.setRotation(q);
		    	br.sendTransform(tf::StampedTransform(newtransform, ros::Time::now(), "odom", "wrench"));
		    }
		    catch (tf2::TransformException &ex) {
				ROS_WARN("%s",ex.what());
				ros::Duration(0.1).sleep();
				continue;
		    }

		    rate.sleep();
		}
	}
	return 0;
}