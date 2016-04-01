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

//CV_Bridge Variables
static const std::string OPENCV_WINDOW = "Kinect Image";
static const std::string CAMERA_TOPIC = "/front_kinect/depth/image_raw";

//Canny Variables
cv::Mat src;

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
    image_sub_ = it_.subscribe(CAMERA_TOPIC, 1, 
    &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_processor/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //get image cv::Pointer
    cv_bridge::CvImagePtr cv_ptr;

    //acquire image frame
    try
    {
      cv::convertScaleAbs(msg, src, 100, 0.0);
      //cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  //src = cv_ptr->image;

  /// Wait until user exit program by pressing a key
  cv::waitKey(3);

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processor");
  ImageConverter ic;
  ros::spin();
  return 0;
}
