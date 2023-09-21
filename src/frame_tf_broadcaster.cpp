#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>

/*
	TODO : Haptic frame moves correctly in the camera optical frame, and the listener changes the incoming position so moving forward moves the robot away from camera and left or right correctly. I was told to fix the controller frames 'base' and 'tool0' since they arent linked correctly, and make a middle link like checkerboard between them and camera. The way I had it before would not work if camera was rotated in any way. Test with different camera orientation to make sure it works.
	
	I've tested a the camera to the left of where the robot faces, and rotated the camera to face the camera, and the robot moved accordingly. 
*/


geometry_msgs::PoseStamped pose_data;
omni_msgs::OmniButtonEvent button_data;
bool point_received = false;
bool button_received = false;

void button_callback(const omni_msgs::OmniButtonEvent &  _data){
	// read the pose of the robot
	button_data = _data;
	button_received = true;
}

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
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = node.subscribe("/phantom/phantom/button", 1, button_callback);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  bool signal = false;
  float x;
  float y;
  float z;

  ros::Rate rate(10.0);

  while (node.ok()){
  	if (button_received && !signal){
  		if (button_data.white_button == 1){
  			signal = true;
  		}
  	}
  	
  	if (point_received && signal){
  		x = pose_data.pose.position.x;
  		y = pose_data.pose.position.y;
  		z = pose_data.pose.position.z;
  		//transform.setOrigin(tf::Vector3(x + (ros::Time::now().toSec()), y + (ros::Time::now().toSec()), z + (ros::Time::now().toSec())));
			transform.setOrigin( tf::Vector3(x, -z, y) ); 
		  //transform.setOrigin( tf::Vector3(0, 0, 0) );
		  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link_optical", "haptic"));
		}
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
