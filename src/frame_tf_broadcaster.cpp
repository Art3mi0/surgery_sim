#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <surgery_sim/HapConfigConfig.h>
#include <surgery_sim/Reset.h>
#include <cmath>
#include <std_msgs/Int32.h>


geometry_msgs::PoseStamped pose_data;
std_msgs::Int32 degree_data;
float theta = 120;
float pi = atan(1) * 4;
bool point_received = false;
bool degree_received = false;

void callback(surgery_sim::HapConfigConfig &config, uint32_t level) {
  theta = config.degree_param * (pi / 180);
}

void get_pose(const geometry_msgs::PoseStamped &  _data){
	// read the pose of the haptic device
	pose_data = _data;

	point_received = true;
}

// void degree_callback(const std_msgs::Int32 &  _data){
// 	// read the pose of the haptic device
// 	theta = _data.data * (pi / 180);

// 	degree_received = true;
// }

int main(int argc, char* argv[]){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

	// dynamic_reconfigure::Server<surgery_sim::Testcfg> server;
  // dynamic_reconfigure::Server<surgery_sim::Testcfg>::CallbackType f;

  // f = boost::bind(&callback, _1, _2);
  // server.setCallback(f);
 
  // subscriber for reading the haptid device pose
	ros::Subscriber haptic_sub = node.subscribe("/phantom/phantom/pose", 1, get_pose);
	// ros::Subscriber degree_sub = node.subscribe("/degree", 1, degree_callback);

	dynamic_reconfigure::Server<surgery_sim::HapConfigConfig> server;
  dynamic_reconfigure::Server<surgery_sim::HapConfigConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  float x;
  float y;
  float z;

  ros::Rate rate(10.0);

  while (node.ok()){ 	
  	if (point_received){
			ROS_INFO("%f\n", theta);
  		x = pose_data.pose.position.x;
  		y = pose_data.pose.position.y;
  		z = pose_data.pose.position.z;
  		//transform.setOrigin(tf::Vector3(x + (ros::Time::now().toSec()), y + (ros::Time::now().toSec()), z + (ros::Time::now().toSec())));
			//transform.setOrigin( tf::Vector3(x, -z, y) );  // x, -z, y
			//transform.setOrigin( tf::Vector3(x, 0.6428*y-0.766*z , 0.766*y+0.6428*z) );  // error
			//transform.setOrigin( tf::Vector3(x, -1*(0.766*y+0.6428*z), -1*(-.6428*y+0.766*z)));  // works
			//TODO: convert to radians
			transform.setOrigin( tf::Vector3(x, cos(theta)*y-sin(theta)*z, sin(theta)*y+cos(theta)*z));  // 120 degrees is the sweet spot
		  //transform.setOrigin( tf::Vector3(0, 0, 0) );
		  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_user", "haptic"));
		}
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
