#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>


geometry_msgs::PoseStamped pose_data;
omni_msgs::OmniButtonEvent button_data;
bool point_received = false;
bool button_received = false;

void button_callback(const omni_msgs::OmniButtonEvent &  _data){
	// read the pose of the robot
	button_data = _data;
	button_received = true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "plan_tf_broadcaster");
  ros::NodeHandle node;
 
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = node.subscribe("/phantom/phantom/button", 1, button_callback);

  tf::TransformBroadcaster br;
  tf::Transform transform;
  
  bool signal = false;

  ros::Rate rate(10.0);

  while (node.ok()){
  	if (button_received && !signal){
  		if (button_data.white_button == 1){
  			signal = true;
  		}
  	}
  	
			transform.setOrigin( tf::Vector3(0, 0, 0) ); 
		  transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dummy_link", "plan"));
		  
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
