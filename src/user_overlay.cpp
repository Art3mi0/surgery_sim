#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

static const std::string OPENCV_WINDOWL = "Image window left";
static const std::string OPENCV_WINDOWR = "Image window right";
cv_bridge::CvImagePtr cv_ptr_l;
cv_bridge::CvImagePtr cv_ptr_r;
bool flagL = false;
bool flagR = false;
std::string text = "Current Mode:";

void imageCbL(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr_l = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr_l->image.rows > 60 && cv_ptr_l->image.cols > 60)
    cv::putText(cv_ptr_l->image, 
    text,
    cv::Point(cv_ptr_l->image.cols/2, 50), 
    cv::FONT_HERSHEY_DUPLEX,
    1.0, 
    CV_RGB(255,0,0),
    2);

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOWL, cv_ptr_l->image);
  //cv::waitKey(3);

  flagL = true;
}

void imageCbR(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr_r = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (cv_ptr_r->image.rows > 60 && cv_ptr_r->image.cols > 60)
    cv::putText(cv_ptr_r->image, 
    text,
    cv::Point(cv_ptr_r->image.cols/2, 50), 
    cv::FONT_HERSHEY_DUPLEX,
    1.0, 
    CV_RGB(255,0,0),
    2);

  // Update GUI Window
  //cv::imshow(OPENCV_WINDOWR, cv_ptr_r->image);
  //cv::waitKey(3);

  flagR = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "user_overlay");
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_transport::Subscriber subL = it.subscribe("/stereo/left/image_raw", 1, imageCbL);
  image_transport::Subscriber subR = it.subscribe("/stereo/right/image_raw", 1, imageCbR);
  image_transport::Publisher pubL = it.advertise("/overlay_left/output_video", 1);
  image_transport::Publisher pubR = it.advertise("/overlay_right/output_video", 1);

  int loop_freq = 60;
  ros::Rate loop_rate(loop_freq);
  
  while (node.ok()){
    if (flagL){
      pubL.publish(cv_ptr_l->toImageMsg());
    }
    if (flagR){
      pubR.publish(cv_ptr_r->toImageMsg());
    }

  loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}