#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
//#include <omni_msgs/OmniButtonEvent.h>
#include <surgery_sim/Plan.h>
#include <surgery_sim/Reset.h>
#include <cstdlib>
#include<omni_msgs/OmniFeedback.h>
#include "std_msgs/Int32.h"

std_msgs::Int32 button_data;
geometry_msgs::Twist plan_point;
geometry_msgs::Twist haptic_point;
geometry_msgs::Twist robot_point;
bool plan_received = false;
bool haptic_received = false;
bool button_received = false;
bool h_pose_received = false;
bool robot_received = false;
bool timer_white = false;
bool timer_grey = false;
bool update_flag;
bool white_press;
bool grey_press;
int click_count = 0;

void button_callback(const std_msgs::Int32 &  _data){
	// read the button input
	button_data = _data;
	button_received = true;
}

void plan_callback(const geometry_msgs::Twist &  _data){
	// read the generated plan
	plan_point = _data;
	plan_received = true;
}

void haptic_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the haptic device
	haptic_point = _data;
	haptic_received = true;
}

void robot_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the robot
	robot_point = _data;
	robot_received = true;
}

geometry_msgs::PoseStamped phantom_pos;

void get_phantom_pos(const geometry_msgs::PoseStamped & _data){
	phantom_pos = _data;
	h_pose_received = true;
}

// desired values of x, y, z for centering the haptic device
double x_d = -0.007;
double y_d = -.043;
double z_d = 0.046;

// ry = hx divide r by 3 rz = hz r/3 -rx=hy

// proportional and derivate control gains for the x,y,z axis of the haptic device when tracking a certain reference position of the robot or centering them
double kp_x = 25.0;
double kd_x = 20.0;
double kp_y = 25.0;
double kd_y = 20.0;
double kp_z = 50.0;
double kd_z = 20.0;


double e_x_prev = 0.0, e_y_prev = 0.0, e_z_prev = 0.0;


// current and previous centering forces for the haptic device
omni_msgs::OmniFeedback centering_force;
omni_msgs::OmniFeedback centering_force_prev;

//PD tracker for the position of the haptic device
void calc_center_force(void){
	double e_x, e_y, e_z, de_x, de_y, de_z;
	//calculate the error
  e_x = x_d - phantom_pos.pose.position.x; 
	e_y = y_d - phantom_pos.pose.position.y; 
	e_z = z_d - phantom_pos.pose.position.z; 
        //calculate the derivatives of the errors
        de_x = e_x - e_x_prev;
	de_y = e_y - e_y_prev;
	de_z = e_z - e_z_prev;
        //low pass filter for reducing the noise and jerks in the forces
	double tau = 0.8;
	centering_force.force.x = (kp_x*e_x + kd_x*de_x)*(1-tau) + tau*centering_force_prev.force.x;
	centering_force.force.y = (kp_y*e_y + kd_y*de_y)*(1-tau) + tau*centering_force_prev.force.y;
  centering_force.force.z = (kp_z*e_z + kd_z*de_z)*(1-tau) + tau*centering_force_prev.force.z + 0.2;//the last component is for the effect hand weight
	// reduce the kicks by resetting th force feedbacks when the reference changes
 	e_x_prev = e_x;
	e_y_prev = e_y;
	e_z_prev = e_z;
	centering_force_prev = centering_force;
}

void timer_callback(const ros::TimerEvent& event){
	if (white_press){
		timer_white = true;
	} else{
		timer_grey = true;
	}
}

void update_force(double x, double y, double z){
	x_d = y/3;
	y_d = -x;
	z_d = z/3;
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
  
  ros::Timer timer = node.createTimer(ros::Duration(1), timer_callback, true);
 
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = node.subscribe("/key", 1, button_callback);
	// Subscriber for reading the plan trajectory
	ros::Subscriber plan_sub = node.subscribe("/refplan", 1, plan_callback);
	// Subscriber for reading the haptic trajectory
	ros::Subscriber haptic_sub = node.subscribe("/refhap", 1, haptic_callback);
	// Subscriber for reaping the haptic device pose
	ros::Subscriber haptic_pos_sub = node.subscribe("/phantom/phantom/pose",10, get_phantom_pos);
	// Subscriber for reaping the robot pose
	ros::Subscriber robot_pos_sub = node.subscribe("/ur5e/toolpose",10, robot_callback);
	// Publisher for the kinematic solver. Skips the plan point generation
  ros::Publisher pub_robot = node.advertise<geometry_msgs::Twist>("/reftraj", 1);
  // Publisher for the force feedback of haptic device
  ros::Publisher force_pub =node.advertise<omni_msgs::OmniFeedback>("/phantom/phantom/force_feedback",1);
  
  bool white_flag;
  bool grey_flag;
  bool robot_flag = true;
  
  double rx;
	double ry;
	double rz;
  
  geometry_msgs::Twist robot_initial;
  
  ros::Rate rate(10.0);

  while (node.ok()){
  	if (robot_received){
  		if(robot_flag){
				robot_initial = robot_point;
				robot_flag = false;
			}else{
				rx = (robot_point.linear.x - robot_initial.linear.x)*.5;
				ry = (robot_point.linear.y - robot_initial.linear.y);
				rz = (robot_point.linear.z - robot_initial.linear.z);
			}
  	}
  	
  	//left-white middle-grey
  	//Left pedal-a	Middle pedal-b Right pedal-c
  	if (button_received){
  		if (button_data.data == 97 && (click_count == 0 || white_flag)){
  			white_flag = false;
  			grey_flag = true;
  			white_press = true;
  			grey_press = false;
  			timer_grey = false;
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
    		timer.stop();
    		timer.setPeriod(ros::Duration(.2));
    		timer.start();
  		}
  		else if (button_data.data == 98 && (click_count == 0 || grey_flag)){
  			white_flag = true;
  			grey_flag = false;
  			white_press = false;
  			grey_press = true;
  			timer_white = false;
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
    		timer.stop();
    		timer.setPeriod(ros::Duration(.5));
    		timer.start(); 		
  		}
  			
  		if (timer_white && plan_received){
  			update_force(rx, ry, rz);
  			pub_robot.publish(plan_point);
			} else if (timer_grey && haptic_received){
				pub_robot.publish(haptic_point);
			}
		}
		//if (h_pose_received && robot_received){
			calc_center_force();
			force_pub.publish(centering_force);
			/*
			ROS_INFO("Differences: x:%f y:%f z:%f", phantom_pos.pose.position.x - ry/3, phantom_pos.pose.position.y - -rx, phantom_pos.pose.position.z - rz/3);
		}*/
		
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
