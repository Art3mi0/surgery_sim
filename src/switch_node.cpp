#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <surgery_sim/Plan.h>
#include <surgery_sim/Reset.h>
#include <cstdlib>

omni_msgs::OmniButtonEvent button_data;
geometry_msgs::Twist plan_point;
geometry_msgs::Twist haptic_point;
bool plan_received = false;
bool haptic_received = false;
bool button_received = false;
int click_count = 0;

void button_callback(const omni_msgs::OmniButtonEvent &  _data){
	// read the pose of the robot
	button_data = _data;
	button_received = true;
}

void plan_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the robot
	plan_point = _data;
	plan_received = true;
}

void haptic_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the robot
	haptic_point = _data;
	haptic_received = true;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "switch_node");
  ros::NodeHandle node;
  
  // initializing server interaction
  ros::ServiceClient traj_client = node.serviceClient<surgery_sim::Reset>("reset");
  ros::ServiceClient haptic_client = node.serviceClient<surgery_sim::Reset>("haptic_server");
  ros::ServiceClient frame_client = node.serviceClient<surgery_sim::Reset>("frame_server");
  ros::ServiceClient plan_client = node.serviceClient<surgery_sim::Reset>("plan_server");
  surgery_sim::Reset reset;
 
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = node.subscribe("/phantom/phantom/button", 1, button_callback);
	// Subscriber for reading the plan trajectory
	ros::Subscriber plan_sub = node.subscribe("/refplan", 1, plan_callback);
	// Subscriber for reading the haotic trajectory
	ros::Subscriber haptic_sub = node.subscribe("/refhap", 1, haptic_callback);
	// Publisher for the kinematic solver. Skips the plan point generation
  ros::Publisher pub_robot = node.advertise<geometry_msgs::Twist>("/reftraj", 1);
  // Publisher for the plan of points
  //ros::Publisher plan_pub = node.advertise<surgery_sim::Plan>("/plan", 1);
  
  bool white_press;
  bool grey_press;
  
  ros::Rate rate(10.0);

  while (node.ok()){
  	if (button_received){
  		if (button_data.white_button == 1){
  			white_press = true;
  			grey_press = false;
  			reset.request.plan_start = true;
  			reset.request.plan_flag = true;
  			reset.request.hap_start = false;
  			reset.request.hap_flag = false;
  			haptic_client.call(reset);
  			frame_client.call(reset);
  			if (click_count > 0){
  				plan_client.call(reset);
  			} else{
  				traj_client.call(reset);
  			}
    		ROS_INFO("out: started plan. haptic off");
    		haptic_received = false;
    		click_count ++;
  		}
  		else if (button_data.grey_button == 1){
  			white_press = false;
  			grey_press = true;
  			reset.request.plan_flag = false;
  			reset.request.plan_start = false;
  			reset.request.hap_start = true;
  			reset.request.hap_flag = true;
  			traj_client.call(reset);
  			plan_client.call(reset);
  			haptic_client.call(reset);
  			//frame_client.call(reset);
    		ROS_INFO("out: resetting plan. haptic on");
    		plan_received = false;
    		click_count ++; 			
  		}
  			
  		if (white_press && plan_received){
  			pub_robot.publish(plan_point);
			} else if (grey_press && haptic_received){
				pub_robot.publish(haptic_point);
			}
		}
		
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
/*
reset.request.flag = true;
  			if (client.call(reset))
  			{
    			ROS_INFO("out: %s", (bool)reset.response.out ? "true" : "false");
  			} */
