#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

static const std::string OPENCV_WINDOWL = "Image window left";
static const std::string OPENCV_WINDOWR = "Image window right";
static const int RADIUS = 10;
cv_bridge::CvImagePtr cv_ptr_l;
cv_bridge::CvImagePtr cv_ptr_r;
image_geometry::PinholeCameraModel cam_model_l;
image_geometry::PinholeCameraModel cam_model_r;
bool flagL = false;
bool flagR = false;
bool pcl_received = false;
std::string text = "Current Mode:";
pcl::PointCloud<pcl::PointXYZ> plan_cloud;

void imageCbL(const sensor_msgs::ImageConstPtr& image_msg_l,
const sensor_msgs::CameraInfoConstPtr& info_msg_l)
{
  try
  {
    cv_ptr_l = cv_bridge::toCvCopy(image_msg_l, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cam_model_l.fromCameraInfo(info_msg_l);

  // Draw an example circle on the video stream
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

void imageCbR(const sensor_msgs::ImageConstPtr& image_msg_r,
const sensor_msgs::CameraInfoConstPtr& info_msg_r)
{
  try
  {
    cv_ptr_r = cv_bridge::toCvCopy(image_msg_r, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cam_model_r.fromCameraInfo(info_msg_r);

  // Draw an example circle on the video stream
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

void pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, plan_cloud);
	pcl_received = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "user_overlay");
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);

  ros::Subscriber pcl_sub = node.subscribe("/plancloud", 1, pcl_callback);
  // ros::Publisher pcl_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/testcloud", 1);

// should subscribe to image_rect_color when using real cameras. Need to run image_proc for this topic
  image_transport::CameraSubscriber subL = it.subscribeCamera("/stereo/left/image_raw", 1, imageCbL);
  image_transport::CameraSubscriber subR = it.subscribeCamera("/stereo/right/image_raw", 1, imageCbR);
  image_transport::Publisher pubL = it.advertise("/overlay_left/output_video", 1);
  image_transport::Publisher pubR = it.advertise("/overlay_right/output_video", 1);

  tf::TransformListener listener;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_l;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_r;
  int loop_freq = 60;
  ros::Rate loop_rate(loop_freq);
  
  while (node.ok()){
     if (pcl_received){ 
      tf::StampedTransform transform;
      geometry_msgs::Transform transform2;
      try
      {
        pcl_ros::transformPointCloud(cam_model_l.tfFrame(), plan_cloud, cloud_out_l, listener);
        pcl_ros::transformPointCloud(cam_model_r.tfFrame(), plan_cloud, cloud_out_r, listener);
        for (int i = 0; i < cloud_out_l.points.size(); i++){
          cv::Point3d pt_cv_l(cloud_out_l.points[i].x, cloud_out_l.points[i].y, cloud_out_l.points[i].z);
          cv::Point3d pt_cv_r(cloud_out_r.points[i].x, cloud_out_r.points[i].y, cloud_out_r.points[i].z);
          cv::Point2d uv_l;
          cv::Point2d uv_r;
          uv_l = cam_model_l.project3dToPixel(pt_cv_l);
          uv_r = cam_model_r.project3dToPixel(pt_cv_r);

          cv::circle(cv_ptr_l->image, uv_l, RADIUS, CV_RGB(255,0,0), -1);
          cv::circle(cv_ptr_r->image, uv_r, RADIUS, CV_RGB(255,0,0), -1);
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    // if (flag){
    //   pcl_pub.publish(cloud_out);
    // }
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