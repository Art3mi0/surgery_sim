#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <surgery_sim/Reset.h>


/*
	TODO remove adding a frame to the plan to work again. The commented out code shuold all be fine, what isnt was for demonstration. Try to go back and not make an extra frame, or see if you can make a transform without broadcasting. To get the pointcloud to work, I think change the frame id, or do some frame transformation like in his code on git.
	
	What I had originally was fine, except creating a broadcaster and using the wrong frame for the header of the debug pcl. I also didn't double check the plan (was modifying x when I should have y).
*/

geometry_msgs::Twist robot_initial;
surgery_sim::Reset reset;
surgery_sim::Plan plan;
std::vector<geometry_msgs::Twist> plan_points;
pcl::PointCloud<pcl::PointXYZI> dbg_cloud;
bool robot_received = false;
bool plan_created = false;
bool call_traj = false;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_initial = _data;
	robot_received = true;
}

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  bool pos_found = false;
  std::vector<geometry_msgs::Twist> new_plan_points;
  if (req.plan_flag){
  plan_created = false;
  //ROS_INFO("the y of first point: %f", plan_points[0].linear.y);
  	for (int i = 0; i < plan_points.size() - 1; i ++) {
  		if (pos_found){
  			new_plan_points.push_back(plan_points[i + 1]);
  		}else if ((robot_initial.linear.y >= plan_points[i].linear.y) && (robot_initial.linear.y <= plan_points[i + 1].linear.y)){
  			new_plan_points.push_back(robot_initial);
  			new_plan_points.push_back(plan_points[i + 1]);
  			pos_found = true;
  		}
  	}
  plan.points = new_plan_points;
  plan_created = true;
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
  // publisher for writing the plan
	ros::Publisher pub_plan= node.advertise<surgery_sim::Plan>( "/plan", 1);

  tf::TransformListener listener;
  
  // Initiating variables
  bool robot_recorded = false;
  geometry_msgs::Twist initial;
  
  geometry_msgs::Twist plan_point;

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
