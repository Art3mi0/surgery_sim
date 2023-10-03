#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>


/*
	TODO remove adding a frame to the plan to work again. The commented out code shuold all be fine, what isnt was for demonstration. Try to go back and not make an extra frame, or see if you can make a transform without broadcasting. To get the pointcloud to work, I think change the frame id, or do some frame transformation like in his code on git.
	
	What I had originally was fine, except creating a broadcaster and using the wrong frame for the header of the debug pcl. I also didn't double check the plan (was modifying x when I should have y).
*/

geometry_msgs::Twist robot_initial;
pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
bool robot_received = false;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_initial = _data;
	robot_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "plan_listener");

  ros::NodeHandle node;

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
  // Publisher for the RViz visual
  ros::Publisher pub_point= node.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );
  ros::Publisher dbg_traj_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("plancloud",1);
  // publisher for writing the plan
	ros::Publisher pub_plan= node.advertise<surgery_sim::Plan>( "/plan", 1);

  tf::TransformListener listener;
  
  // Initiating variables
  bool robot_recorded = false;
  bool plan_created = false;
  geometry_msgs::Twist initial;
  
  surgery_sim::Plan plan;
  geometry_msgs::Twist plan_point;
  std::vector<geometry_msgs::Twist> plan_points;

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
    tf::StampedTransform transform;
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
		
		// Records the initial position of the robot once
		if (robot_received && !robot_recorded){
			initial = robot_initial;
			robot_recorded = true;
			plan_points.push_back(initial);
			
			plan_point.angular.x = initial.angular.x;
			plan_point.angular.y = initial.angular.y;
			plan_point.angular.z = initial.angular.z;
		}
		
		if (robot_recorded && !plan_created){
			dbg_cloud.points.clear();
			pcl::PointXYZI tmp;
			tmp.x = initial.linear.x;
			tmp.y = initial.linear.y;
			tmp.z = initial.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x();
			plan_point.linear.y = transform.getOrigin().y() - .1;
			plan_point.linear.z = initial.linear.z;
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = initial.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan_point.linear.x = transform.getOrigin().x();
			plan_point.linear.y = transform.getOrigin().y() - .1;
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
			
			plan_point.linear.x = transform.getOrigin().x();
			plan_point.linear.y = transform.getOrigin().y() + .1;
			plan_point.linear.z = transform.getOrigin().z();
			plan_points.push_back(plan_point);
			tmp.x = plan_point.linear.x;
			tmp.y = plan_point.linear.y;
			tmp.z = plan_point.linear.z;
			dbg_cloud.points.push_back(tmp);
			
			plan.points = plan_points;	
			plan_created = true;
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
		}

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
