#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <surgery_sim/PedalEvent.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include<sensor_msgs/PointCloud2.h>
#include <surgery_sim/Reset.h>
#include <cmath>
#include "std_msgs/Int32.h"

#define PI 3.14159265

/*
This node creates a plan that will be sent to the task_space_traj_reset node. The different types of plans are
a hard coded one, 5 based off of models, and 1 based off a path from the surgical path planning package. For
the path from the path planning, you would run the node from that package, create a path using the pen, then
when content with the path you hit the right pedal and this node makes a plan. Path planning functionality
may be broken.

After creating a plan, this node will handle removing points from the overlay. It can do this because it also 
sends a pointcloud, so when the tool gets close enough to a point, the point in the pcl gets removed. It refreshes
the pcl when it swaps to autonmous. It does not remove points when in autonomous.
*/

geometry_msgs::Twist robot_current;
surgery_sim::Reset reset;
surgery_sim::Plan plan;
surgery_sim::Plan robot_plan;
std::vector<geometry_msgs::Twist> plan_points;
std::vector<geometry_msgs::Twist> robot_plan_points;
pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
pcl::PointCloud<pcl::PointXYZI> dbg_rob_cloud;
pcl::PointCloud<pcl::PointXYZI> robot_cloud;
pcl::PointCloud<pcl::PointXYZ> filtered_path;
pcl::PointCloud<pcl::PointXYZ> transformed_path;
bool robot_received = false;
bool plan_created = false;
bool call_traj = false;
bool dbg_flag = false;
bool filter_received = false;
bool pedal_received = false;
surgery_sim::PedalEvent pedal_data;

std::string plan_type = "model"; // Options: coded; clicked; planner; model

bool custom_plan = true;
bool hap_flag;
float offset = .0015;

void filtered_callback(const sensor_msgs::PointCloud2 & _data){   
     pcl::fromROSMsg(_data, filtered_path);
     filter_received = true;
}

void pedal_callback(const surgery_sim::PedalEvent &  _data){
	// read the pedal input
	pedal_data = _data;
	pedal_received = true;
}

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_current = _data;
	robot_received = true;
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

/*
This method is called when a mode swap occurs. This refreshes the pcl.
*/
void update_dbg(){
	pcl::PointCloud<pcl::PointXYZI> fresh_dbg_cloud;
	dbg_cloud.points.clear();
	fresh_dbg_cloud.points.clear();
	pcl::PointXYZI tmp;
  std::cout << "Back TO Autonomous" << std::endl;
	for (int i = 0; i < plan_points.size(); i ++){
		tmp.x = plan_points[i].linear.x;
		tmp.y = plan_points[i].linear.y;
		tmp.z = plan_points[i].linear.z;
		fresh_dbg_cloud.points.push_back(tmp);
	}
	dbg_cloud = fresh_dbg_cloud;
}

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  std::vector<geometry_msgs::Twist> new_plan_points;

	if (req.hap_flag){
		hap_flag = true;
	} else{
		hap_flag = false;
		update_dbg();
	}

  if (req.plan_flag){
  reset.request.plan_start = true;
  reset.request.plan_flag = true;
  call_traj = true;
  }	
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "plan_listener");

  ros::NodeHandle node;
	ros::NodeHandle home("~");
	std::string plan_type = "model_ew";
	home.getParam("plan_type", plan_type); // options are: coded; clicked; planner; model
  
  ros::ServiceServer service = node.advertiseService("plan_server", flag);
  ros::ServiceClient traj_client = node.serviceClient<surgery_sim::Reset>("reset");

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
  // Publisher for the RViz visual
  ros::Publisher pub_point= node.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );
  ros::Publisher dbg_traj_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("plancloud",1);
	ros::Publisher dbg_traj_rob_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("robot_plancloud",1);
	// subscriber for reading pedal input
	ros::Subscriber pedal_sub = node.subscribe("/pedal", 1, pedal_callback);
	// subscriber for reading pcl from path planner
	ros::Subscriber filtered_sub = node.subscribe("/filtered_path", 1, filtered_callback);

  // Needed robot toolpoint when running simulation without model
  ros::Publisher rob_point_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("robot_point",1);
  // publisher for writing the plan
	ros::Publisher pub_plan= node.advertise<surgery_sim::Plan>( "/plan", 1);
	ros::Publisher pub_rob_plan= node.advertise<surgery_sim::Plan>( "/robot_plan", 1);

  tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::StampedTransform transform_cube;

	float cube_length = .0099;
	float cylinder_length = .025;
  
  // Initiating variables
  bool robot_recorded = false;
  geometry_msgs::Twist initial;
  geometry_msgs::Twist plan_point;
	float rob_x;
	float rob_y;
	float rob_z;

	pcl::PointXYZI tmp_rob;

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
	if ((plan_type == "model_ew") || (plan_type == "model_ns") || (plan_type == "model_we") || (plan_type == "model_sn")){
		try{
		// When transforming from one thing that already exists to something else, 
		// it is not necessary to make a broadcaster node. Just use the existing links.
		listener.lookupTransform("base", "dummy_link_ew",  
								ros::Time(0), transform);
		}
		catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	} else if (plan_type == "train"){
		try{
		listener.lookupTransform("base", "cube_training",  
								ros::Time(0), transform_cube);
		}
		catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
	} 
		
	// Records the initial position of the robot once
	if (robot_received && !robot_recorded){
		initial = robot_current;
		robot_recorded = true;
		
		plan_point.angular.x = initial.angular.x;
		plan_point.angular.y = initial.angular.y;
		plan_point.angular.z = initial.angular.z;
	}

	// Handles publishing current robot point for rviz visualization
	if (robot_received){
		robot_cloud.points.clear();
		tmp_rob.x = robot_current.linear.x;
		tmp_rob.y = robot_current.linear.y;
		tmp_rob.z = robot_current.linear.z;
		robot_cloud.push_back(tmp_rob);

		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("base");
		robot_cloud.header = pcl_conversions::toPCL(header);
		rob_point_pub.publish(robot_cloud);
	}
	
	if (robot_recorded && !plan_created){
		dbg_cloud.points.clear();
		pcl::PointXYZI tmp;

		tmp.x = initial.linear.x;
		tmp.y = initial.linear.y;
		tmp.z = initial.linear.z;

		// Will create plan based off an object added in gazebo. Will not work without object.
		if (plan_type == "model_ew"){			
			plan_point.linear.x = transform.getOrigin().x() + .015;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = initial.linear.z;
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;
			float tmp_off = 0.0;

			for (int i = 0; i < 7; i++){
				plan_point.linear.x = transform.getOrigin().x() + (.015 + tmp_off);
				plan_point.linear.y = transform.getOrigin().y();
				plan_point.linear.z = transform.getOrigin().z() + .0025;
				plan_points.push_back(plan_point);
				tmp.x = plan_point.linear.x;
				tmp.y = plan_point.linear.y;
				tmp.z = plan_point.linear.z;
				dbg_cloud.points.push_back(tmp);
				tmp_off = tmp_off - 0.005;
			}

			plan.points = plan_points;	
			plan_created = true;

		}else if (plan_type == "model_ns"){			
			plan_point.linear.x = transform.getOrigin().x();
			plan_point.linear.y = transform.getOrigin().y() + .015;
			plan_point.linear.z = initial.linear.z;
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;			
			float tmp_off = 0.0;

			for (int i = 0; i < 7; i++){
				plan_point.linear.x = transform.getOrigin().x();
				plan_point.linear.y = transform.getOrigin().y() + (.015 + tmp_off);
				plan_point.linear.z = transform.getOrigin().z() + .0025;
				plan_points.push_back(plan_point);
				tmp.x = plan_point.linear.x;
				tmp.y = plan_point.linear.y;
				tmp.z = plan_point.linear.z;
				dbg_cloud.points.push_back(tmp);
				tmp_off = tmp_off - 0.005;
			}

			plan.points = plan_points;	
			plan_created = true;

		} else if (plan_type == "model_we"){			
			plan_point.linear.x = transform.getOrigin().x() + .015;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = initial.linear.z;
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;	
			float tmp_off = 0.0;

			for (int i = 0; i < 7; i++){
				plan_point.linear.x = transform.getOrigin().x() - (.015 + tmp_off);
				plan_point.linear.y = transform.getOrigin().y();
				plan_point.linear.z = transform.getOrigin().z() + .0025;
				plan_points.push_back(plan_point);
				tmp.x = plan_point.linear.x;
				tmp.y = plan_point.linear.y;
				tmp.z = plan_point.linear.z;
				dbg_cloud.points.push_back(tmp);
				tmp_off = tmp_off - 0.005;
			}

			plan.points = plan_points;	
			plan_created = true;

		}else if (plan_type == "model_sn"){			
			plan_point.linear.x = transform.getOrigin().x() + .015;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = initial.linear.z;
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;			
			float tmp_off = 0.0;

			for (int i = 0; i < 7; i++){
				plan_point.linear.x = transform.getOrigin().x();
				plan_point.linear.y = transform.getOrigin().y() - (.015 + tmp_off);
				plan_point.linear.z = transform.getOrigin().z() + .0025;
				plan_points.push_back(plan_point);
				tmp.x = plan_point.linear.x;
				tmp.y = plan_point.linear.y;
				tmp.z = plan_point.linear.z;
				dbg_cloud.points.push_back(tmp);
				tmp_off = tmp_off - 0.005;
			}

			plan.points = plan_points;	
			plan_created = true;

		}else if (plan_type == "train"){			
			// Creates path around cube
			plan_point.linear.x = transform_cube.getOrigin().x();
			plan_point.linear.y = transform_cube.getOrigin().y();
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = transform_cube.getOrigin().x() + cube_length;
			plan_point.linear.y = transform_cube.getOrigin().y() + cube_length;
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = transform_cube.getOrigin().x() + cube_length;
			plan_point.linear.y = transform_cube.getOrigin().y() - cube_length;
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = transform_cube.getOrigin().x() - cube_length;
			plan_point.linear.y = transform_cube.getOrigin().y() - cube_length;
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = transform_cube.getOrigin().x() - cube_length;
			plan_point.linear.y = transform_cube.getOrigin().y() + cube_length;
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = transform_cube.getOrigin().x() - cylinder_length;
			plan_point.linear.y = transform_cube.getOrigin().y();
			plan_point.linear.z = transform_cube.getOrigin().z() + .005;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			float tmp_pi = 0;

			// Creates circular path around cylinder object
			for (int i = 0; i < 12; i++){
				plan_point.linear.x = transform_cube.getOrigin().x() - cylinder_length + sin(tmp_pi) * 0.01;
				plan_point.linear.y = transform_cube.getOrigin().y() + cos(tmp_pi) * 0.01;
				plan_point.linear.z = transform_cube.getOrigin().z() + .005;
				plan_points.push_back(plan_point);
				tmp.x = plan_point.linear.x;
				tmp.y = plan_point.linear.y;
				tmp.z = plan_point.linear.z;
				dbg_cloud.points.push_back(tmp);
				tmp_pi = tmp_pi + PI/6;
			}			

			plan.points = plan_points;	
			plan_created = true;

		}else if (plan_type == "coded"){
			float z_offset = -.006;

			plan_point.linear.x = 0.050777;
			plan_point.linear.y = -0.58775;
			plan_point.linear.z = 0.0378 + z_offset;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = 0.0495;
			plan_point.linear.y = -0.594;
			plan_point.linear.z = 0.037 + z_offset;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = 0.049;
			plan_point.linear.y = -0.5996;
			plan_point.linear.z = 0.037 + z_offset;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = 0.0502;
			plan_point.linear.y = -0.605767;
			plan_point.linear.z = 0.034 + z_offset;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan.points = plan_points;	
			plan_created = true;
		} else if ((plan_type == "planner") && (filter_received)){
			if ((pedal_received) && (pedal_data.right_pedal == 1)){
				pcl_ros::transformPointCloud("base", filtered_path, transformed_path, listener);
				
				for (int i = 0; i < transformed_path.points.size(); i++){
					plan_point.linear.x = transformed_path.points[i].x;
					plan_point.linear.y = transformed_path.points[i].y;
					plan_point.linear.z = transformed_path.points[i].z;
					plan_points.push_back(plan_point);
					tmp.x = plan_point.linear.x;
					tmp.y = plan_point.linear.y;
					tmp.z = plan_point.linear.z;
					dbg_cloud.points.push_back(tmp);
					
					plan_point.linear.z = transformed_path.points[i].z + .004;
					robot_plan_points.push_back(plan_point);
					tmp.x = plan_point.linear.x;
					tmp.y = plan_point.linear.y;
					tmp.z = plan_point.linear.z;
					dbg_rob_cloud.points.push_back(tmp);
				}
				plan.points = plan_points;
				robot_plan.points = robot_plan_points;
				plan_created = true;
		}
	}
	}
	
	// Only works when in manual mode. Removes points from pcl. The pcl is used by user_overlay
	if (hap_flag){
		rob_x = robot_current.linear.x;
		rob_y = robot_current.linear.y;
		rob_z = robot_current.linear.z;
		for (int i = 0; i < dbg_cloud.size(); i++){
			if ((calc_diff(rob_x, dbg_cloud[i].x)) && 
			(calc_diff(rob_y, dbg_cloud[i].y)) && 
			(calc_diff(rob_z, dbg_cloud[i].z))){
				dbg_cloud.erase(dbg_cloud.begin() + i);		
			}
		}
	}

	if (plan_created){
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("base");
		dbg_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_pub.publish(dbg_cloud);

		header.stamp = ros::Time::now();
		dbg_rob_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_rob_pub.publish(dbg_rob_cloud);

		pub_plan.publish(plan);
		pub_rob_plan.publish(robot_plan);
		if (call_traj){
			traj_client.call(reset);
			call_traj = false;
		}
	}

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
};