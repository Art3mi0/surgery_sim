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
#include <string>
#include <surgery_sim/PedalEvent.h>


/*
This node subscribes to the image_raw topics of the cameras and modifies the images. It can resize, crop, resize
and crop, or display the full image.
When publishing the image, be wary of the resulting MP. It can't be greater than your ethernet transfer speed or
else there will be a larger delay between images in the other computer/device.
We use the exact same cameras, so we do not have to worry about capture rates and syncing the images. The images
we recieve are published at a rate of 40hz, so publishing at 30hz is able to remain consistant.
The images have text at the top stating the current mode, what the starting mode will be, a timer, and projects
points from a poincloud. When the experiment starts, the starting mode text is replaced with a timer.
The pcl comes form the plan listener. This node subscribes to the point counter node so that it can color the
correct target marker differently.

Currently the scales are for specific parameters. The view for the resized and cropped function has only been 
made to look good with the real robot, and the crop function has only been made to look good with the simulator.
Images will look off with anyother combination. The projection should be fine for any case.

We only display the text on one view because it did not look good during development on both views. I don't 
remeber what specifically was wrong with it. Also, I believe the text would look better if it was positioned
at the bottom of the screen. At the top, the images start looking a bit off, but they remain consistantly good
at the bottom.
*/

static const int RADIUS = 5; // old size was 7
int text_size = 3;
int height;
int width;
int comp_size = 0;
std::string text = "Standby";
std::string next_text = "Starting In Autonomous";
geometry_msgs::Twist robot_point;
surgery_sim::Plan robot_plan;
surgery_sim::Plan user_plan;

int crop_x;
int crop_y;
int crop_width;
int crop_height;

int comp_tmp = 0;
int plan_tmp = 0;
int color_fix = 0;

int r = 0;    // for 1 instance of text
int g = 0;
int b = 0;

int r2 = 255; // for "starting in" text
int g2 = 255;
int b2 = 255;

int dr = 255; // for depth text color
int dg = 255;
int db = 255;

int cr = 0; // for manual vs auto lines
int cg = 255;
int cb = 0;

int pr = 0; // for points
int pg = 0;
int pb = 255;

float depth;

cv_bridge::CvImagePtr cv_ptr_l;
cv_bridge::CvImagePtr cv_ptr_r;
image_geometry::PinholeCameraModel cam_model_l;
image_geometry::PinholeCameraModel cam_model_r;

bool show_next = true;
bool flagL = false;
bool flagR = false;
bool pcl_received = false;
bool robot_received = false;
bool got_confidence = false;
pcl::PointCloud<pcl::PointXYZ> plan_cloud;
pcl::PointCloud<pcl::PointXYZ> robot_plan_cloud;
// pcl::PointCloud<pcl::PointXYZ> user_plan_cloud;
pcl::PointCloud<pcl::PointXYZ> test_cloud;
pcl::PointCloud<pcl::PointXYZI> confidence_cloud;

int right_count = 0;
bool held = false;

void pedal_callback(const surgery_sim::PedalEvent & _data){
	if (_data.right_pedal == 1){
			if (!held){
					held = true;
					right_count++;
          if (right_count == 2){
            text = "Ready To Start";
          }
			}
	} else{
			held = false;
	}
}

// when the mode switches, the text and text color will also change
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

void get_rob_plan(const surgery_sim::Plan & _data){
  robot_plan = _data;
}

void get_user_plan(const surgery_sim::Plan & _data){
  user_plan = _data;
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

void robot_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, robot_plan_cloud);
	pcl_received = true;
}
// void user_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
// 	pcl::fromROSMsg(_data, user_plan_cloud);
// 	pcl_received = true;
// }

void conf_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, confidence_cloud);
  got_confidence = true;
}

void robot_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the robot
	robot_point = _data;
	robot_received = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "user_overlay");
  ros::NodeHandle node;
  ros::NodeHandle home("~");
	std::string source = "plan";
  std::string mode = "crop";
  std::string start_mode = "manual";
  bool sim = true;
  bool dbg = false;
	home.getParam("sim", sim); // options are: "true"; "false"
  home.getParam("dbg", dbg); // options are: "true"; "false"
	home.getParam("source", source); // options are: "click"; "plan"; "path"
  home.getParam("mode", mode); // options are: "resize"; "crop"; "full"; "crop_resize"
  home.getParam("start_mode", start_mode); // options are : manual; auto

  if (start_mode == "manual"){
    next_text = "Starting In Manual";
  }
  
  image_transport::ImageTransport it(node);
  ros::ServiceServer service = node.advertiseService("overlay_server", flag);

  std::string subscriber_topic = "/user_plancloud";
  if (source == "click"){
    subscriber_topic = "/test_cloud";
  } else if (source == "path"){
    subscriber_topic = "/filtered_path";
  }
  if (!sim){
    crop_x = 867;
    crop_y = 720;
    crop_width = 1733;
    crop_height = 1440;
  } else{
    crop_x = 400;
    crop_y = 550;
    crop_width = 500;
    crop_height = 500;
  }

  cv::Rect crop(crop_x, crop_y, crop_width, crop_height);

  ros::Subscriber conf_points_sub = node.subscribe("/path_confidence" ,1, conf_pcl_callback);
  ros::Subscriber robot_points_sub = node.subscribe("/robot_plancloud" ,1, robot_pcl_callback);
  // ros::Subscriber user_points_sub = node.subscribe("/user_plancloud" ,1, user_pcl_callback);
  ros::Subscriber completed_sub = node.subscribe("/completed_points" ,1, get_completed);
  ros::Subscriber rob_plan_sub = node.subscribe("/robot_plan" ,1, get_rob_plan);
  ros::Subscriber user_plan_sub = node.subscribe("/user_plan" ,1, get_user_plan);
  ros::Subscriber pcl_sub = node.subscribe(subscriber_topic, 1, pcl_callback); // cloud from chosen source
  ros::Subscriber robot_pos_sub = node.subscribe("/ur5e/toolpose",10, robot_callback);
  ros::Publisher pcl_l_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/overlay_cloud_l", 1);
  ros::Publisher pcl_r_pub = node.advertise<pcl::PointCloud<pcl::PointXYZ> >("/overlay_cloud_r", 1);
  ros::Subscriber pedal_sub = node.subscribe("/pedal", 1, pedal_callback);

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
  std::string time_str;
  std::string depth_str;

  cv::Point2d prev_pt_l;
  cv::Point2d prev_pt_r;
  int prev;

  tf::TransformListener listener;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_l;
  pcl::PointCloud<pcl::PointXYZ>  cloud_out_r;
  pcl::PointCloud<pcl::PointXYZ>  rob_cloud_out_l;
  pcl::PointCloud<pcl::PointXYZ>  rob_cloud_out_r;
  pcl::PointCloud<pcl::PointXYZI>  conf_cloud_out_l;
  pcl::PointCloud<pcl::PointXYZI>  conf_cloud_out_r;
  cv::Mat resized_down_l;
  cv::Mat resized_down_r;
  bool got_transform = false;
  bool got_time = false;
  int loop_freq = 30;
  ros::Rate loop_rate(loop_freq);
  
  // In the point projection, the points from the first pointcloud are from the surface of an object, and if it
  // is the real robot, the second are from an offset.
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
        if (dbg){ // dbg needs to be changed to a better name. dbg means using the projected error
          // From Planner
          // pcl_ros::transformPointCloud(cam_model_l.tfFrame(), robot_plan_cloud, rob_cloud_out_l, listener);
          // pcl_ros::transformPointCloud(cam_model_r.tfFrame(), robot_plan_cloud, rob_cloud_out_r, listener);

          // From Projected error node
          pcl_ros::transformPointCloud(cam_model_l.tfFrame(), confidence_cloud, conf_cloud_out_l, listener);
          pcl_ros::transformPointCloud(cam_model_r.tfFrame(), confidence_cloud, conf_cloud_out_r, listener);
        } else {
          // set dbg false then uncomment this for exact robot point. Do this for calibration and checking if accurate.
          pcl_ros::transformPointCloud(cam_model_l.tfFrame(), robot_plan_cloud, cloud_out_l, listener);
          pcl_ros::transformPointCloud(cam_model_r.tfFrame(), robot_plan_cloud, cloud_out_r, listener);

          // Uncomment this for surface points. Only set of code from this else section can be uncommented at a time.
          // pcl_ros::transformPointCloud(cam_model_l.tfFrame(), plan_cloud, cloud_out_l, listener);
          // pcl_ros::transformPointCloud(cam_model_r.tfFrame(), plan_cloud, cloud_out_r, listener);
        }
        
        got_transform = true;

        // If the real robot, it will do what it just did but with the extra pcl.
        if (dbg){
          // change between conf_cloud_out_l/r and rob_cloud_out_l/r
          for (int i = 1; i < conf_cloud_out_l.points.size(); i++){
            prev = i - 1;
            cv::Point3d rob_pt_cv_l(conf_cloud_out_l.points[i].x, conf_cloud_out_l.points[i].y, conf_cloud_out_l.points[i].z);
            cv::Point3d rob_pt_cv_r(conf_cloud_out_r.points[i].x, conf_cloud_out_r.points[i].y, conf_cloud_out_r.points[i].z);
            cv::Point2d rob_uv_l;
            cv::Point2d rob_uv_r;
            cv::Point2d l_point;
            cv::Point2d r_point;
            rob_uv_l = cam_model_l.project3dToPixel(rob_pt_cv_l);
            rob_uv_r = cam_model_r.project3dToPixel(rob_pt_cv_r);
            l_point = cam_model_l.unrectifyPoint(rob_uv_l);
            r_point = cam_model_r.unrectifyPoint(rob_uv_r);

            cv::Point3d rob_pt_cv_l2(conf_cloud_out_l.points[prev].x, conf_cloud_out_l.points[prev].y, conf_cloud_out_l.points[prev].z);
            cv::Point3d rob_pt_cv_r2(conf_cloud_out_r.points[prev].x, conf_cloud_out_r.points[prev].y, conf_cloud_out_r.points[prev].z);
            cv::Point2d rob_uv_l2;
            cv::Point2d rob_uv_r2;
            cv::Point2d l_point2;
            cv::Point2d r_point2;
            rob_uv_l2 = cam_model_l.project3dToPixel(rob_pt_cv_l2);
            rob_uv_r2 = cam_model_r.project3dToPixel(rob_pt_cv_r2);
            l_point2 = cam_model_l.unrectifyPoint(rob_uv_l2);
            r_point2 = cam_model_r.unrectifyPoint(rob_uv_r2);

            if ((i < conf_cloud_out_l.points.size())){
              if ((confidence_cloud.points[prev].intensity == 0) && (start_mode == "auto")){
                cv::line(cv_ptr_l->image, l_point2, l_point, CV_RGB(cr, cg, cb), 2);
                cv::line(cv_ptr_r->image, r_point2, r_point, CV_RGB(cr, cg, cb), 2);

                cv::circle(cv_ptr_l->image, l_point, 4, CV_RGB(pr,pg,pb), -1);
                cv::circle(cv_ptr_r->image, r_point, 4, CV_RGB(pr,pg,pb), -1);
                if ((i == 1)){
                  cv::circle(cv_ptr_l->image, l_point2, 4, CV_RGB(250,150,250), -1);
                  cv::circle(cv_ptr_r->image, r_point2, 4, CV_RGB(250,150,250), -1);
                } else{
                  cv::circle(cv_ptr_l->image, l_point2, 4, CV_RGB(pr,pg,pb), -1);
                  cv::circle(cv_ptr_r->image, r_point2, 4, CV_RGB(pr,pg,pb), -1);
                }
              } else if (start_mode == "manual"){
                cv::circle(cv_ptr_l->image, l_point, 4, CV_RGB(pr,pg,pb), -1);
                cv::circle(cv_ptr_r->image, r_point, 4, CV_RGB(pr,pg,pb), -1);

                cv::circle(cv_ptr_l->image, l_point2, 4, CV_RGB(pr,pg,pb), -1);
                cv::circle(cv_ptr_r->image, r_point2, 4, CV_RGB(pr,pg,pb), -1);
              }
            }
          }
        } else{
          for (int i = 0; i < cloud_out_l.points.size(); i++){
            cv::Point3d pt_cv_l(cloud_out_l.points[i].x, cloud_out_l.points[i].y, cloud_out_l.points[i].z);
            cv::Point3d pt_cv_r(cloud_out_r.points[i].x, cloud_out_r.points[i].y, cloud_out_r.points[i].z);
            cv::Point2d uv_l;
            cv::Point2d uv_r;
            cv::Point2d test1;
            cv::Point2d test2;

            uv_l = cam_model_l.project3dToPixel(pt_cv_l);
            uv_r = cam_model_r.project3dToPixel(pt_cv_r);
            // I'm pretty sure the points are projected to a rectified image, and unrectifying the points improve
            // precision of points
            test1 = cam_model_l.unrectifyPoint(uv_l);
            test2 = cam_model_r.unrectifyPoint(uv_r);

            if (i == comp_size - color_fix){
              cv::circle(cv_ptr_l->image, test1, RADIUS, CV_RGB(0,255,0), -1);
              cv::circle(cv_ptr_r->image, test2, RADIUS, CV_RGB(0,255,0), -1);
            } else{
              cv::circle(cv_ptr_l->image, test1, RADIUS, CV_RGB(255,0,0), -1);
              cv::circle(cv_ptr_r->image, test2, RADIUS, CV_RGB(255,0,0), -1);
            }
          }
        }

        pcl_l_pub.publish(cloud_out_l);
        pcl_r_pub.publish(cloud_out_r);
      }
      catch (tf::TransformException ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
    }

    if ((flagL) && (flagR) && (got_transform)){
      // set mode at the top of the program or in the launch file
      if (mode == "resize"){
        cv::resize(cv_ptr_l->image, resized_down_l, cv::Size(width*.3, height*.3), CV_INTER_LINEAR);
        cv::resize(cv_ptr_r->image, resized_down_r, cv::Size(width*.3, height*.3), CV_INTER_LINEAR);

        cv::putText(resized_down_l, 
        text,
        cv::Point((resized_down_l.cols/2) - 280, 50),
        cv::FONT_HERSHEY_DUPLEX,
        text_size * 0.5, 
        CV_RGB(255,0,0),
        2);

        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_l).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_r).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      } else if (mode == "crop_resize") {
        cv::putText(cv_ptr_l->image, 
        text,
        cv::Point((crop.width) / 1.1, crop.height * .55), 
        cv::FONT_HERSHEY_DUPLEX,
        text_size, 
        CV_RGB(255,0,0),
        2);

        if (show_next){
          cv::putText(cv_ptr_l->image, 
          next_text,
          cv::Point((crop.width / 1.2), crop.height * .6), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.75, 
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
          time_str = std::to_string(tmp_time);
          time_str.resize(time_str.size()-5);
          
          cv::putText(cv_ptr_l->image, 
          time_str + "s",
          cv::Point((crop.width / 1.1), crop.height * .6), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.75, 
          CV_RGB(255, 255, 0),
          2);
          
          //Begin depth display
          depth = robot_point.linear.z - robot_plan.points[comp_size].linear.z;
          depth = depth * 1000; // Converts to mm
          if (depth >= 1){          // White if freater than 1mm
            dr = 255;
            dg = 255;
            db = 255;
          } else if (depth >= .5){  // Light green if close above
            dr = 90;
            dg = 240;
            db = 50;
          } else if (depth >= -.5){  // Green if close
            dr = 0;
            dg = 120;
            db = 0;
          } else if (depth >= -1){   // Yellow if close below
            dr = 255;
            dg = 255;
            db = 0;
          } else{   // Red if too deep
            dr = 255;
            dg = 0;
            db = 0;
          }
          depth_str = std::to_string(depth);
          depth_str.resize(depth_str.size()-4);
          cv::putText(cv_ptr_l->image, 
          depth_str + "mm",
          cv::Point((crop.width / .9), crop.height * .8), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.75, 
          CV_RGB(dr, dg, db),
          2);
        }
        //End depth display

        cv::resize(cv_ptr_l->image(crop), resized_down_l, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        cv::resize(cv_ptr_r->image(crop), resized_down_r, cv::Size(width*.45, height*.45), CV_INTER_LINEAR);
        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_l).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resized_down_r).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      } else if (mode == "crop") {
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
          time_str = std::to_string(tmp_time);
          time_str.resize(time_str.size()-5);
          cv::putText(cv_ptr_l->image, 
          time_str + "s",
          cv::Point((crop.width / .88), crop.height * 1.21), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.35, 
          CV_RGB(255, 255, 0),
          2);

          //Begin depth display
          depth = robot_point.linear.z - user_plan.points[comp_size].linear.z;
          depth = depth * 1000; // Converts to mm
          if (depth >= 1){          // White if freater than 1mm
            dr = 255;
            dg = 255;
            db = 255;
          } else if (depth >= .5){  // Light green if close above
            dr = 90;
            dg = 240;
            db = 50;
          } else if (depth >= -.5){  // Green if close
            dr = 0;
            dg = 120;
            db = 0;
          } else if (depth >= -1){   // Yellow if close below
            dr = 255;
            dg = 255;
            db = 0;
          } else{   // Red if too deep
            dr = 255;
            dg = 0;
            db = 0;
          }
          depth_str = std::to_string(depth);
          depth_str.resize(depth_str.size()-4);
          cv::putText(cv_ptr_l->image, 
          depth_str + "mm",
          cv::Point((crop.width / .88), crop.height * 1.41), 
          cv::FONT_HERSHEY_DUPLEX,
          text_size * 0.35, 
          CV_RGB(dr, dg, db),
          2);
          //End depth display
        }

        l_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_l->image(crop)).toImageMsg();
        r_img_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr_r->image(crop)).toImageMsg();

        pubL.publish(l_img_ptr);
        pubR.publish(r_img_ptr);

      } else{   // Full size
        cv::putText(cv_ptr_l->image, 
        text,
        cv::Point((800/6), 800 * .07), 
        cv::FONT_HERSHEY_DUPLEX,
        text_size * .5, 
        CV_RGB(255,0,0),
        3);

        pubL.publish(cv_ptr_l->toImageMsg());
        pubR.publish(cv_ptr_r->toImageMsg());
      }
    }

  loop_rate.sleep();
  ros::spinOnce();
  }
  return 0;
}