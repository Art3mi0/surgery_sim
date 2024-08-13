#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <std_msgs/Bool.h>


/*
This node acts as a memory for the autonomous node and user overlay. This keeps track of what points have been
passed. The other nodes just check size, so if the plan is not followed correctly, bugs may occur
*/

geometry_msgs::Twist robot_current;
surgery_sim::Plan plan;
surgery_sim::Plan completed_points;
std::vector<geometry_msgs::Twist> temp_points;
std_msgs::Bool stop_auto;
bool robot_received = false;
bool complete_flag = true;
bool plan_received = false;
bool got_confidence = false;

float offset = .001;
geometry_msgs::Twist dummy_point;

pcl::PointCloud<pcl::PointXYZI> confidence_cloud;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_current = _data;
	robot_received = true;
}

void plan_callback(const surgery_sim::Plan & _data){
	plan = _data;
	plan_received = true;
}

void conf_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, confidence_cloud);
  got_confidence = true;
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

float calc_distance(geometry_msgs::Twist p, geometry_msgs::Twist q){
	float x;
	float y;
	float z;

	x = pow(p.linear.x - q.linear.x, 2);
	y = pow(p.linear.y - q.linear.y, 2);
	z = pow(p.linear.z - q.linear.z, 2);

	return pow(x + y + z, 0.5);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "point_counter");

  ros::NodeHandle node;
	ros::NodeHandle home("~");

	bool sim = true;
	home.getParam("sim", sim); // options are: "true"; "false"

	std::string subscriber_topic = "/plan";

	if (!sim){
		subscriber_topic = "/robot_plan";
	}

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
	ros::Subscriber pub_sub= node.subscribe( subscriber_topic, 1, plan_callback);
	ros::Subscriber conf_points_sub = node.subscribe("/path_confidence" ,1, conf_pcl_callback);

	// publisher for writing the plan
	ros::Publisher pub_completed= node.advertise<surgery_sim::Plan>( "/completed_points", 1);
	ros::Publisher pub_stop= node.advertise<std_msgs::Bool>( "/stop_auto", 1);

  tf::TransformListener listener;
  
  // Initiating variables
	bool index = false;
  bool robot_recorded = false;
	bool once = true;
	stop_auto.data = false;
  geometry_msgs::Twist initial;
  
	float rob_x;
	float rob_y;
	float rob_z;
	float shortest_distance = 999;
	float tmp_distance;
	int shortest_point = 0;
	int current_point;

	dummy_point.linear.x = 0.07;
	dummy_point.linear.y = 0.018;
	dummy_point.linear.z = 0.404;

	pcl::PointXYZI tmp_rob;

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){

	if (robot_received){
		rob_x = robot_current.linear.x;
		rob_y = robot_current.linear.y;
		rob_z = robot_current.linear.z;

		if (once){
			dummy_point.angular.x = robot_current.angular.x;
			dummy_point.angular.y = robot_current.angular.y;
			dummy_point.angular.z = robot_current.angular.z;
			once = false;
		}
	}
	
	if (index){

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
				}
			}
		}
		
		completed_points.points = temp_points;
		pub_completed.publish(completed_points);

	} else{
		if ((plan.points.size() > 0) && (plan_received)){
			for (int i = 0; i < plan.points.size(); i++){
				tmp_distance = calc_distance(plan.points[i], robot_current);
				if (tmp_distance < shortest_distance){
					shortest_distance = tmp_distance;
					shortest_point = i;
				}
			}
			//std::cout << "Distance to closest point: " << shortest_distance << " the shortest point is: " << shortest_point+1 << std::endl;
			while (temp_points.size() != shortest_point){
				if (temp_points.size() < shortest_point){
					temp_points.push_back(dummy_point);
				} else{
					temp_points.pop_back();
				}
			}
			completed_points.points = temp_points;
			pub_completed.publish(completed_points);
		}
	}

	current_point = completed_points.points.size();
	if (got_confidence){
		if (confidence_cloud.points[current_point].intensity == 1){
			//std::cout << "Distance to closest point: " << tmp_distance << std::endl;
			if (shortest_distance < .0001){
				stop_auto.data = true;
			} else{
				stop_auto.data = false;
			}
		}
	}

	shortest_distance = 999;

	pub_stop.publish(stop_auto);

	loop_rate.sleep();
	ros::spinOnce();
  }
  return 0;
};