#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <surgery_sim/Reset.h>


geometry_msgs::Twist robot_current;
pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
float offset = .002;
bool robot_received = false;


void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_current = _data;
	robot_received = true;
}

void point_callback(const geometry_msgs::PointStamped & _data){
	pcl::PointXYZI tmp;
	tmp.x = _data.point.x;
	tmp.y = _data.point.y;
	tmp.z = _data.point.z;
	dbg_cloud.push_back(tmp);
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
  ros::init(argc, argv, "overlay_broadcast");

  ros::NodeHandle node;
  
	ros::Subscriber point_sub = node.subscribe("/clicked_point", 1, point_callback);
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
  ros::Publisher dbg_traj_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("test_cloud",1);

	float rob_x;
	float rob_y;
	float rob_z;

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);

  while (node.ok()){
		if (robot_received){
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

  	std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("base");
		dbg_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_pub.publish(dbg_cloud);
				
		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
