#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <surgery_sim/Reset.h>


geometry_msgs::PoseStamped pose_data;
bool point_received = false;
bool send_pos = false;

void get_pose(const geometry_msgs::PoseStamped &  _data){
	// read the pose of the haptic device
	pose_data = _data;

	point_received = true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
 
  // subscriber for reading the haptid device pose
	ros::Subscriber haptic_sub = node.subscribe("/phantom/phantom/pose", 1, get_pose);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  float x;
  float y;
  float z;

  ros::Rate rate(10.0);

  while (node.ok()){ 	
  	if (point_received){
  		x = pose_data.pose.position.x;
  		y = pose_data.pose.position.y;
  		z = pose_data.pose.position.z;
  		//transform.setOrigin(tf::Vector3(x + (ros::Time::now().toSec()), y + (ros::Time::now().toSec()), z + (ros::Time::now().toSec())));
			transform.setOrigin( tf::Vector3(x, -z, y) ); 
		  //transform.setOrigin( tf::Vector3(0, 0, 0) );
		  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_user", "haptic"));
		}
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
