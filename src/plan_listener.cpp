#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <surgery_sim/Reset.h>
#include <cmath>


/*

*/

geometry_msgs::Twist robot_current;
surgery_sim::Reset reset;
surgery_sim::Plan plan;
std::vector<geometry_msgs::Twist> plan_points;
pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
pcl::PointCloud<pcl::PointXYZI> robot_cloud;
bool robot_received = false;
bool plan_created = false;
bool call_traj = false;
bool dbg_flag = false;

bool custom_plan = true;
bool hap_flag;
float offset = .005;

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
		// std::cout << "Updated point "<<i+1 << std::endl;
		// std::cout << "Current size "<<fresh_dbg_cloud.size() << std::endl;
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
		dbg_flag = true;
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
  
  ros::ServiceServer service = node.advertiseService("plan_server", flag);
  ros::ServiceClient traj_client = node.serviceClient<surgery_sim::Reset>("reset");

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
  // Publisher for the RViz visual
  ros::Publisher pub_point= node.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );
  ros::Publisher dbg_traj_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("plancloud",1);

  // Needed robot toolpoint when running simulation without model
  ros::Publisher rob_point_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("robot_point",1);
  // publisher for writing the plan
	ros::Publisher pub_plan= node.advertise<surgery_sim::Plan>( "/plan", 1);

  tf::TransformListener listener;
  
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
	tf::StampedTransform transform;
	if (!custom_plan){
		try{
		// Self note: When transforming from one thing that already exists to something else, 
		// it is not necessary to make a broadcaster node. Just use the existing links.
		listener.lookupTransform("base", "dummy_link",  
								ros::Time(0), transform);
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
		plan_points.push_back(initial);
		
		plan_point.angular.x = initial.angular.x;
		plan_point.angular.y = initial.angular.y;
		plan_point.angular.z = initial.angular.z;
	}

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
		dbg_cloud.points.push_back(tmp);

		// Will create plan based off an object added in gazebo. Will not work without object.
		if (!custom_plan){			
			plan_point.linear.x = transform.getOrigin().x() + .1;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = initial.linear.z;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x() + .1;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x() + .05;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x();
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x() - .05;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x() - .1;
			plan_point.linear.y = transform.getOrigin().y();
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan.points = plan_points;	
			plan_created = true;

		// Custom points chosen by moving the robot and copying pose.
		} else{
			plan_point.linear.x = 0.0461;
			plan_point.linear.y = -0.574775;
			plan_point.linear.z = 0.051236;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = 0.0387;
			plan_point.linear.y = -0.6211;
			plan_point.linear.z = 0.05129;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan_point.linear.x = -0.0109;
			plan_point.linear.y = -0.6156;
			plan_point.linear.z = 0.05127;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);

			plan.points = plan_points;	
			plan_created = true;
		}
	}
	
	if (hap_flag){
		rob_x = robot_current.linear.x;
		rob_y = robot_current.linear.y;
		rob_z = robot_current.linear.z;
		for (int i = 0; i < dbg_cloud.size(); i++){
			if ((calc_diff(rob_x, dbg_cloud[i].x)) && 
			(calc_diff(rob_y, dbg_cloud[i].y)) && 
			(calc_diff(rob_z, dbg_cloud[i].z))){
				dbg_cloud.erase(dbg_cloud.begin() + i);	
				// std::cout << "DELETED point "<<i+1 << std::endl;
				// std::cout << "Current size "<<dbg_cloud.size() << std::endl;			
			}
		}
	}

	if (dbg_flag){
		update_dbg();
		dbg_flag = false;
	}

	if (plan_created){
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		// When trying to tag a frame to something that used transformed data from, make sure
		// the tagged frame is the frame the data was transformed into.
		header.frame_id = std::string("base");
		dbg_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_pub.publish(dbg_cloud);
		pub_plan.publish(plan);
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