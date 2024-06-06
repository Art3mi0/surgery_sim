#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <cmath>


/*

*/

geometry_msgs::Twist robot_current;
surgery_sim::Plan plan;
surgery_sim::Plan completed_points;
std::vector<geometry_msgs::Twist> temp_points;
bool robot_received = false;
bool complete_flag = true;
bool plan_received = false;

float offset = .001;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_current = _data;
	robot_received = true;
}

void plan_callback(const surgery_sim::Plan & _data){
	plan = _data;
	plan_received = true;
}

bool sgn(float num1, float num2){
	if ((num1 > 0) && (num2 > 0)){
		return true;
	} else if ((num1 < 0) && (num2 < 0)){
		return true;
	}
	return false;
}

bool calc_diff(float num1, float num2){
	if (sgn){
		if (std::abs(num1 - num2) < offset){
			return true;
		}
		return false;
	}
	if (std::abs(num1 - (-1 * num2)) < offset){
			return true;
		}
		return false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "point_counter");

  ros::NodeHandle node;

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
	ros::Subscriber pub_sub= node.subscribe( "/plan", 1, plan_callback);

	// publisher for writing the plan
	ros::Publisher pub_completed= node.advertise<surgery_sim::Plan>( "/completed_points", 1);

  tf::TransformListener listener;
  
  // Initiating variables
  bool robot_recorded = false;
  geometry_msgs::Twist initial;
  
	float rob_x;
	float rob_y;
	float rob_z;

	pcl::PointXYZI tmp_rob;

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){

	if (robot_received){
		rob_x = robot_current.linear.x;
		rob_y = robot_current.linear.y;
		rob_z = robot_current.linear.z;
	}
	
	if ((plan.points.size() > 0) && (plan_received)){
		for (int i = 0; i < plan.points.size(); i++){
			if ((calc_diff(rob_x, plan.points[i].linear.x)) && 
			(calc_diff(rob_y, plan.points[i].linear.y)) && 
			(calc_diff(rob_z, plan.points[i].linear.z))){
				for (int j = 0; j < temp_points.size(); j ++){
					if (temp_points[j] == plan.points[i]){
						complete_flag = false;
					}
				}
				if (complete_flag){
					temp_points.push_back(plan.points[i]);
				} else{
					complete_flag = true;
				}
				// std::cout << "DELETED point "<<i+1 << std::endl;
				// std::cout << "Current size "<<dbg_cloud.size() << std::endl;			
			}
		}
	}
	
	completed_points.points = temp_points;
	pub_completed.publish(completed_points);

	loop_rate.sleep();
	ros::spinOnce();
  }
  return 0;
};