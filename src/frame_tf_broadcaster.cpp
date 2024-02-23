#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <surgery_sim/Reset.h>

/*
	TODO : Haptic frame moves correctly in the camera optical frame, and the listener changes the incoming position so moving forward moves the robot away from camera and left or right correctly. I was told to fix the controller frames 'base' and 'tool0' since they arent linked correctly, and make a middle link like checkerboard between them and camera. The way I had it before would not work if camera was rotated in any way. Test with different camera orientation to make sure it works.
	
	I've tested a the camera to the left of where the robot faces, and rotated the camera to face the camera, and the robot moved accordingly. 
*/


geometry_msgs::PoseStamped pose_data;
bool point_received = false;
bool send_pos = false;

void get_pose(const geometry_msgs::PoseStamped &  _data){
	// read the pose of the haptic device
	pose_data = _data;

	point_received = true;
}


bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  if (req.hap_flag){
  	send_pos = true;
  	ROS_INFO("request: Initialize Haptic");
  }/* else {
  	send_pos = false;
  	ROS_INFO("request: Paused Haptic");
  }*/
  return true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  
  // initializing server
	//ros::ServiceServer service = node.advertiseService("frame_server", flag);
 
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
			transform.setOrigin( tf::Vector3(x, -y, -z) ); 
		  //transform.setOrigin( tf::Vector3(0, 0, 0) );
		  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link_optical_middle", "haptic"));
		}
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
