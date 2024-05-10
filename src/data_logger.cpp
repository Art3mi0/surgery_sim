#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <surgery_sim/PedalEvent.h>

geometry_msgs::Twist pose_current;
bool pose_received = false;
bool stop = false;


void pose_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	pose_current = _data;
	pose_received = true;
}

void pedal_callback(const surgery_sim::PedalEvent &  _data){
	// Read the input of a pedal
	if (_data.right_pedal == 1){
    stop = true;
  }
}

int main(int argc, char * argv[]){
	
	// define the ros node
	ros::init(argc,argv, "data_logger");
	ros::NodeHandle nh;
	ros::NodeHandle home("~");
	

	std::string test_no = "dbg";
	home.getParam("test_no", test_no);



  tf::TransformListener listener;
	ros::Subscriber pose_sub = nh.subscribe("/ur5e/toolpose",1, pose_callback);
  ros::Subscriber pedal_sub = nh.subscribe("/pedal",1, pedal_callback);

  std::ofstream positions_file;
  // positions_file.open(("/home/hsaeidi/ident_data/"+test_no+"_frames.csv").c_str());
  ROS_INFO("Logging...");
  positions_file.open(("/home/temo/pose_data/"+test_no+"_pose.txt").c_str());
  positions_file << "x,y,z,r,p,y" << std::endl;

  int loop_freq = 30;
  ros::Rate loop_rate(loop_freq);
  
  while ((nh.ok()) && (!stop)){		 
    
    if (pose_received){
      positions_file << pose_current.linear.x<< ","<< pose_current.linear.y << ","<< pose_current.linear.z
      << ","<< pose_current.angular.x<< ","<< pose_current.angular.y << ","<< pose_current.angular.z<<std::endl;
    } 
  
    loop_rate.sleep();
    ros::spinOnce();
	}

	ROS_INFO("!!!!!!!Finished logging data =>>> shutting down!!!!!!!");
	return 0;

}