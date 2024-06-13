#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/PedalEvent.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <surgery_sim/Plan.h>
#include <omni_msgs/OmniFeedback.h>
#include <dynamic_reconfigure/server.h>
#include <surgery_sim/HapConfigConfig.h>
#include <std_msgs/Int32.h>


geometry_msgs::Twist pose_current;
geometry_msgs::Twist traj_current;
geometry_msgs::Twist haptic_pose;
geometry_msgs::PoseStamped phantom_pos;
geometry_msgs::PointStamped tool_point;
geometry_msgs::PointStamped tool_point_cam;
geometry_msgs::PointStamped hap_point;
geometry_msgs::PointStamped hap_point_cam;
geometry_msgs::PointStamped traj_point;
geometry_msgs::PointStamped traj_point_cam;
omni_msgs::OmniFeedback phantom_ff;
pcl::PointCloud<pcl::PointXYZ> plan_cloud;
pcl::PointCloud<pcl::PointXYZ> cam_l_cloud;
pcl::PointCloud<pcl::PointXYZ> cam_r_cloud;
surgery_sim::Plan completed_points;
surgery_sim::Plan plan;

int pedal = 0;
int mode = 1;
int theta;

bool mode_received = false;
bool pose_received = false;
bool traj_received = false;
bool pcl_received = false;
bool caml_received = false;
bool camr_received = false;
bool completed_point_received = false;
bool haptic_received = false;
bool h_pose_received = false;
bool h_ff_received = false;

bool stop = false;
bool pcl_flag = true;
bool start = false;
bool config_init = true;
bool once_flag = true;

// Reuses hap config. Does not use int parameter
void callback(surgery_sim::HapConfigConfig &config, uint32_t level) {
  if (config_init){
    config.start = false;
    config.stop = false;
    config_init = false;
  }
  if (!start){
    start = config.start;
  }
  if (!stop){
    stop = config.stop;
  }
}

void get_phantom_pos(const geometry_msgs::PoseStamped & _data){
	phantom_pos = _data;
	h_pose_received = true;
}

void get_phantom_ff(const omni_msgs::OmniFeedback & _data){
	phantom_ff = _data;
	h_ff_received = true;
}

void pose_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	pose_current = _data;
	pose_received = true;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = std::string("base");
  tool_point.header = header;
  tool_point.point.x = pose_current.linear.x;
  tool_point.point.y = pose_current.linear.y;
  tool_point.point.z = pose_current.linear.z;
}

void traj_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	traj_current = _data;
	traj_received = true;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = std::string("base");
  traj_point.header = header;
  traj_point.point.x = traj_current.linear.x;
  traj_point.point.y = traj_current.linear.y;
  traj_point.point.z = traj_current.linear.z;
}

void haptic_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the haptic device
	haptic_pose = _data;
	haptic_received = true;

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = std::string("base");
  hap_point.header = header;
  hap_point.point.x = haptic_pose.linear.x;
  hap_point.point.y = haptic_pose.linear.y;
  hap_point.point.z = haptic_pose.linear.z;
}

void pedal_callback(const surgery_sim::PedalEvent &  _data){
	// Reads and saves the input of the pedal
  // Only reads as if clicked once. Ignores held input
  // Previously controlled start and stop of data logger
  if (_data.right_pedal == 1){
    pedal = 3;
    stop = true;
  } else if (_data.left_pedal == 1){
    //start = true;
    pedal = 1;
  } else if (_data.middle_pedal == 1){
    pedal = 2;
  } else{
    pedal = 0;
  }
}

void pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, plan_cloud);
	pcl_received = true;
}

void caml_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, cam_l_cloud);
	caml_received = true;
}

void camr_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, cam_r_cloud);
	camr_received = true;
}

void mode_callback(const std_msgs::Int32 &  _data){
	// read the pose of the haptic device
	mode = _data.data;

	mode_received = true;
}

// void get_completed(const surgery_sim::Plan & _data){
//   if (_data.points.size() != 0){
//     if (_data.points.size() == plan.points.size()){
//       stop = true;
//     }
//   }
// 	completed_point_received = true;
// }

void plan_callback(const surgery_sim::Plan & _data){
  plan = _data;
}

int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "data_logger");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	

	std::string test_no = "dbg";
	home.getParam("test_no", test_no);

	ros::Subscriber pose_sub = nh.subscribe("/ur5e/toolpose",1, pose_callback);
  ros::Subscriber pedal_sub = nh.subscribe("/pedal",1, pedal_callback);
  ros::Subscriber pcl_sub = nh.subscribe("/plancloud", 1, pcl_callback);
  ros::Subscriber pcl_lcam_sub = nh.subscribe("/overlay_cloud_l", 1, caml_callback);
  ros::Subscriber pcl_rcam_sub = nh.subscribe("/overlay_cloud_r", 1, camr_callback);
  ros::Subscriber plan_sub = nh.subscribe("/plan", 1, plan_callback);
  ros::Subscriber haptic_sub = nh.subscribe("/refhap", 1, haptic_callback);
  ros::Subscriber traj_sub = nh.subscribe("/refplan", 1, traj_callback);
  // ros::Subscriber completed_sub = nh.subscribe("/completed_points" ,1, get_completed);
  ros::Subscriber mode_sub = nh.subscribe("/current_mode", 1, mode_callback);
  ros::Subscriber haptic_pos_sub = nh.subscribe("/phantom/phantom/pose",10, get_phantom_pos);
  ros::Subscriber haptic_ff_sub = nh.subscribe("/phantom/phantom/force_feedback",10, get_phantom_ff);

  dynamic_reconfigure::Server<surgery_sim::HapConfigConfig> server;
  dynamic_reconfigure::Server<surgery_sim::HapConfigConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  pcl::PointCloud<pcl::PointXYZ>  cloud_cam;
  std::ofstream tool_positions_file;
  std::ofstream hap_positions_file;
  tool_positions_file.open(("/home/temo/experiment_data/current_participant/"+test_no+"_poses.csv").c_str());
  tool_positions_file << "tx,ty,tz,tr,tp,ty,tcx,tcy,tcz,hx,hy,hz,hfx,hfy,hfz,hbx,hby,hbz,hcx,hcy,hcz,px,py,pz,pcx,pcy,pcz,t,mode" << std::endl;

  tf::TransformListener listener;
  ros::Time init_time;
  int error_count = 0;
  bool time_flag = true;
  bool transformed_pcl = false;
  float count = 0.0;
  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  
  while ((nh.ok()) && (!stop)){		
    tf::StampedTransform transform_caml;
    tf::StampedTransform transform_camr;

    if (!transformed_pcl && pcl_received){
      try{
        pcl_ros::transformPointCloud("camera_user", plan_cloud, cloud_cam, listener);
        transformed_pcl = true;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    }

    try{
      // listener.lookupTransform("camera_link_pos_left", "base",  
      //                          ros::Time(0), transform_caml);
      // listener.lookupTransform("camera_link_pos_right", "base",  
      //                          ros::Time(0), transform_camr);
      listener.transformPoint("camera_user", tool_point, tool_point_cam);
      listener.transformPoint("camera_user", hap_point, hap_point_cam);
      listener.transformPoint("camera_user", traj_point, traj_point_cam);
    }
    catch (tf::TransformException ex){
      if (error_count % 10 == 0){
        ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
      }
      error_count++;
    }

    //if (pcl_received && pcl_flag && caml_received && camr_received){
    if (pcl_received && pcl_flag && transformed_pcl){
      pcl::io::savePCDFileASCII (("/home/temo/experiment_data/current_participant/"+test_no+"_base.pcd").c_str(), plan_cloud);
	    std::cerr << "Saved " << plan_cloud.size () << " base frame data points " << std::endl;
      // pcl::io::savePCDFileASCII (("/home/temo/experiment_data/current_participant/"+test_no+"_caml.pcd").c_str(), cam_l_cloud);
	    // std::cerr << "Saved " << cam_l_cloud.size () << " camL frame data points " << std::endl;
      // pcl::io::savePCDFileASCII (("/home/temo/experiment_data/current_participant/"+test_no+"_camr.pcd").c_str(), cam_r_cloud);
	    // std::cerr << "Saved " << cam_r_cloud.size () << " camR frame data points " << std::endl;
      pcl::io::savePCDFileASCII (("/home/temo/experiment_data/current_participant/"+test_no+"_usercam.pcd").c_str(), cloud_cam);
	    std::cerr << "Saved " << cloud_cam.size () << " user camera frame data points " << std::endl;
      pcl_flag = false;
    } 
    
    if (pose_received && start){
      if (once_flag){
        ROS_INFO("Logging...");
        once_flag = false;
      }
      if (time_flag){
        init_time = phantom_pos.header.stamp;
        time_flag = false;
      }
      count ++;
      tool_positions_file << pose_current.linear.x<< ","<< pose_current.linear.y << ","<< pose_current.linear.z
      << ","<< pose_current.angular.x<< ","<< pose_current.angular.y << ","<< pose_current.angular.z <<","
      << tool_point_cam.point.x<< ","<< tool_point_cam.point.y << ","<< tool_point_cam.point.z <<","
      << phantom_pos.pose.position.x<< ","<< phantom_pos.pose.position.y << ","<< phantom_pos.pose.position.z
      << ","<< phantom_ff.force.x <<","<< phantom_ff.force.y<< ","<< phantom_ff.force.z << ","
      <<haptic_pose.linear.x<< ","<< haptic_pose.linear.y << ","<< haptic_pose.linear.z
      << "," << hap_point_cam.point.x<< ","<< hap_point_cam.point.y << ","<< hap_point_cam.point.z
      << "," << traj_current.linear.x<< ","<< traj_current.linear.y << ","<< traj_current.linear.z
      << "," << traj_point_cam.point.x<< ","<< traj_point_cam.point.y << ","<< traj_point_cam.point.z
      << ","<< count/loop_freq<< ","<< mode<<std::endl;
    } 
  
    loop_rate.sleep();
    ros::spinOnce();
	}
  tool_positions_file.close();
  // hap_positions_file.close();
	ROS_INFO("!!!!!!!Finished logging data =>>> shutting down!!!!!!!");
	return 0;

}