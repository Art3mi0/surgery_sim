#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left_;
  image_transport::Subscriber image_sub_right_;
  image_transport::Publisher image_pub_left_;
  image_transport::Publisher image_pub_right_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_left_ = it_.subscribe("/stereo/left/image_raw", 1, &ImageConverter::imageCbL, this);
    //image_sub_right_ = it_.subscribe("/stereo/right/image_raw", 1, &ImageConverter::imageCbR, this);
    image_pub_left_ = it_.advertise("/image_converter_left/output_video", 1);
    //image_pub_right_ = it_.advertise("/image_converter_right/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCbL(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr_l;
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
      cv::circle(cv_ptr_l->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr_l->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_left_.publish(cv_ptr_l->toImageMsg());
  }

  void imageCbR(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr_r;
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
      cv::circle(cv_ptr_r->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr_r->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_right_.publish(cv_ptr_r->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}