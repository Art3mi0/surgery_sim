#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>

omni_msgs::OmniButtonEvent button_data;
bool button_received = false;

void button_callback(const omni_msgs::OmniButtonEvent &  _data){
	// read the pose of the robot
	button_data = _data;
	button_received = true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
 
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = node.subscribe("/phantom/phantom/button", 1, button_callback);
	// Publisher for the kinematic solver. Skips the plan point generation
  ros::Publisher pub_robot = node.advertise<geometry_msgs::Twist>("/reftraj", 1);
  // Publisher for the plan of points
  ros::Publisher plan_pub = nh_.advertise<ur5e_control::Plan>("/plan", 1);
  
  bool signal = false;
  
  ros::Rate rate(10.0);

  while (node.ok()){
  	if (button_received){
  		if (button_data.white_button == 1){
  			signal = false;
  		}
  	} else if (button_data.grey_button == 1){
  		signal = true;
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
