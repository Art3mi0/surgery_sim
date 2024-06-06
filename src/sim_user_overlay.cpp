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
#include <surgery_sim/Reset.h>
#include <surgery_sim/Plan.h>
#include <math.h>

static const int RADIUS = 7;
int text_size = 3;
int height;
int width;
int comp_size = 0;
std::string text = "Standby";
std::string next_text = "Starting In Manual";
cv::Rect crop(400, 550, 500, 500);

int comp_tmp = 0;
int plan_tmp = 0;
int color_fix = 0;

int r = 0;
int g = 0;
int b = 0;

int r2 = 255;
int g2 = 255;
int b2 = 255;

cv_bridge::CvImagePtr cv_ptr_l;
cv_bridge::CvImagePtr cv_ptr_r;
image_geometry::PinholeCameraModel cam_model_l;
image_geometry::PinholeCameraModel cam_model_r;

bool show_next = true;
bool flagL = false;
bool flagR = false;
bool pcl_received = false;
pcl::PointCloud<pcl::PointXYZ> plan_cloud;
pcl::PointCloud<pcl::PointXYZ> test_cloud;

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  if (req.preview){
    if (req.manual){
      next_text = "Starting In Manual";
      r2 = 255;
      g2 = 255;
      b2 = 255;
    } else{
      next_text = "Starting In Autonomous";
      r2 = 255;
      g2 = 0;
      b2 = 0;
    }
  } else if (req.hap_start){
    show_next = false;
  	text = "Manual";
    r = 255;
    g = 255;
    b = 255;
    comp_tmp = comp_size;
    plan_tmp = plan_cloud.size();
  }else if (req.plan_start){
    show_next = false;
  	text = "Autonomous";
    r = 255;
    g = 0;
    b = 0;
  } else{
    show_next = false;
    text = "Stopped";
    r = 255;
    g = 0;
    b = 0;
  }
  return true;
}

void get_completed(const surgery_sim::Plan & _data){
  comp_size = _data.points.size();
}

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
  width = cv_ptr_l->image.cols;
  height = cv_ptr_l->image.rows;

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
  ros::NodeHandle home("~");
	std::string source = "plan";
  std::string mode = "full";
	home.getParam("source", source); // options are: "click"; "plan"; "path"
  home.getParam("mode", mode); // options are: "resize"; "crop"; "full"
  image_transport::ImageTransport it(node);

  ros::ServiceServer service = node.advertiseService("overlay_server", flag);

  std::string subscriber_topic = "/plancloud";
  if (source == "click"){
    subscriber_topic = "/test_cloud";
  } else if (source == "path"){
    subscriber_topic = "/filtered_path";
  }

  ros::Subscriber completed_sub = node.subscribe("/completed_points" ,1, get_completed);
  ros::Subscriber pcl_sub = node.subscribe(subscriber_topic, 1, pcl_callback); // cloud from chosen source
  ros::Publisher pcl_l_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/overlay_cloud_l", 1);
  ros::Publisher pcl_r_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/overlay_cloud_r", 1);

// should subscribe to image_rect_color when using real cameras. Need to run image_proc for this topic
  image_transport::CameraSubscriber subL = it.subscribeCamera("/stereo/left/image_raw", 1, imageCbL);
  image_transport::CameraSubscriber subR = it.subscribeCamera("/stereo/right/image_raw", 1, imageCbR);
  image_transport::Publisher pubL = it.advertise("/overlay_left/output_video", 1);
  image_transport::Publisher pubR = it.advertise("/overlay_right/output_video", 1);

  sensor_msgs::ImagePtr l_img_ptr;
  sensor_msgs::ImagePtr r_img_ptr;

  ros::Time start_time;
  ros::Duration delta_time;
  float tmp_time;

  tf::TransformListener listener;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_l;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_r;
  cv::Mat resized_down_l;
  cv::Mat resized_down_r;
  bool got_transform = false;
  bool got_time = false;
  int loop_freq = 30;
  ros::Rate loop_rate(loop_freq);
  
  while (node.ok()){
     if ((pcl_received) && (flagL) && (flagR)){ 
      if (text == "Manual"){
        if ((plan_tmp != plan_cloud.size())){
          color_fix = plan_tmp - plan_cloud.size();;
        }
      } else{
        color_fix = 0;
      }

      tf::StampedTransform transform;
      try
      {
        // listener.lookupTransform("base", "fk_tooltip",  
				// 				ros::Time(0), transform);
        // test_cloud.points.clear();

        pcl_ros::transformPointCloud(cam_model_l.tfFrame(), plan_cloud, cloud_out_l, listener);
        pcl_ros::transformPointCloud(cam_model_r.tfFrame(), plan_cloud, cloud_out_r, listener);
        got_transform = true;
        for (int i = 0; i < cloud_out_l.points.size(); i++){
          cv::Point3d pt_cv_l(cloud_out_l.points[i].x, cloud_out_l.points[i].y, cloud_out_l.points[i].z);
          cv::Point3d pt_cv_r(cloud_out_r.points[i].x, cloud_out_r.points[i].y, cloud_out_r.points[i].z);
          cv::Point2d uv_l;
          cv::Point2d uv_r;
          cv::Point2d test1;
          cv::Point2d test2;
          uv_l = cam_model_l.project3dToPixel(pt_cv_l);
          uv_r = cam_model_r.project3dToPixel(pt_cv_r);
          test1 = cam_model_l.unrectifyPoint(uv_l);
          test2 = cam_model_r.unrectifyPoint(uv_r);

          // uv_l = cam_model_l.unrectifyPoint(uv_l);
          // std::cout<<cloud_out_l.points.size()<< std::endl;


          // Test the unrectified and rectified points after getting a solid 
          // calibration to see if this is improves accuracy

          if (i == comp_size - color_fix){
            cv::circle(cv_ptr_l->image, test1, RADIUS, CV_RGB(0,255,0), -1);
            cv::circle(cv_ptr_r->image, test2, RADIUS, CV_RGB(0,255,0), -1);
          } else{
            cv::circle(cv_ptr_l->image, test1, RADIUS, CV_RGB(255,0,0), -1);
            cv::circle(cv_ptr_r->image, test2, RADIUS, CV_RGB(255,0,0), -1);
          }

          pcl_l_pub.publish(cloud_out_l);
          pcl_r_pub.publish(cloud_out_r);
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    if ((flagL) && (flagR) && (got_transform)){
      // set mode at the top of the program
      if (mode == "resize"){
        cv::resize(cv_ptr_l->image, resized_down_l, cv::Size(width*.3, height*.3), CV_INTER_LINEAR);
        cv::resize(cv_ptr_r->image, resized_down_r, cv::Size(width*.3, height*.3), CV_INTER_LINEAR);

        cv::putText(resized_down_l, 
        text,
        cv::Point((resized_down_l.cols/2) - 280, 50),
        cv::FONT_HERSHEY_DUPLEX,
        text_size * 0.5, 
        CV_RGB(r, g, b),
        2);

        // Don't need both overlays to display text. Result is blurry when at the edge. Would need to
        // find corresponding pixel locations between the views and make the text origin there for 
        // each view to create 3d text I believe.
        // cv::putText(resized_down_r, 
        // text,
        // cv::Point((resized_down_r.cols/2) - 280, 50), 
        // cv::FONT_HERSHEY_DUPLEX,
        // text_size * 0.5, 
        // CV_RGB(255,0,0),
        // 2);

        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_l).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_r).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      } else if (mode == "resize_crop") {
        cv::putText(cv_ptr_l->image, 
        text,
        cv::Point((crop.width/1.3), crop.height * .6), 
        cv::FONT_HERSHEY_DUPLEX,
        text_size * 0.75, 
        CV_RGB(r, g, b),
        2);

        // cv::putText(cv_ptr_r->image, 
        // text,
        // cv::Point((crop.width/2 + 280), crop.height - 220), 
        // cv::FONT_HERSHEY_DUPLEX,
        // text_size * 0.75, 
        // CV_RGB(255,0,0),
        // 2);

        cv::resize(cv_ptr_l->image(crop), resized_down_l, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        cv::resize(cv_ptr_r->image(crop), resized_down_r, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_l).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_r).toImageMsg();

        // l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_l->image(crop)).toImageMsg();
        // r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_r->image(crop)).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      } 
      else if (mode == "crop") {
        cv::putText(cv_ptr_l->image, 
        text,
        cv::Point((crop.width / .88), crop.height * 1.15), 
        cv::FONT_HERSHEY_DUPLEX,
        text_size * 0.35, 
        CV_RGB(r, g, b),
        2);

        if (show_next){
          cv::putText(cv_ptr_l->image, 
          next_text,
          cv::Point((crop.width / 1), crop.height * 1.21), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.35, 
          CV_RGB(r2, g2, b2),
        2);
        } else{
          if (!got_time){
            start_time = ros::Time::now();
            got_time = true;
          }
          if (text != "Stopped"){
            delta_time = ros::Time::now() - start_time;
          }
          tmp_time = delta_time.sec + delta_time.nsec * pow(10, -9);
          cv::putText(cv_ptr_l->image, 
          std::to_string(tmp_time),
          cv::Point((crop.width / .88), crop.height * 1.21), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.35, 
          CV_RGB(255, 255, 0),
        2);
        }

        // cv::putText(cv_ptr_r->image, 
        // text,
        // cv::Point((crop.width/2 + 280), crop.height - 220), 
        // cv::FONT_HERSHEY_DUPLEX,
        // text_size * 0.75, 
        // CV_RGB(255,0,0),
        // 2);

        // cv::resize(cv_ptr_l->image(crop), resized_down_l, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        // cv::resize(cv_ptr_r->image(crop), resized_down_r, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        // l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_l).toImageMsg();
        // r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_r).toImageMsg();

        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_l->image(crop)).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_r->image(crop)).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      }else{
        cv::putText(cv_ptr_l->image, 
        text,
        cv::Point((800/6), 800 * .07), 
        cv::FONT_HERSHEY_DUPLEX,
        text_size * .5, 
        CV_RGB(r, g, b),
        3);

        // cv::putText(cv_ptr_r->image, 
        // text,
        // cv::Point((width/2) - 800, 130), 
        // cv::FONT_HERSHEY_DUPLEX,
        // text_size * 1.5, 
        // CV_RGB(255,0,0),
        // 3);

        pubL.publish(cv_ptr_l->toImageMsg());
        pubR.publish(cv_ptr_r->toImageMsg());
      }
    }

  loop_rate.sleep();
  ros::spinOnce();
  }
  return 0;
}