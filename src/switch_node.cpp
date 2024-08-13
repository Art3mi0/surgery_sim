#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <surgery_sim/Plan.h>
#include <surgery_sim/PedalEvent.h>
#include <surgery_sim/Reset.h>
#include <cstdlib>
#include <omni_msgs/OmniFeedback.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

/*
This node sends robot position data to the ur5e_controller node depending on the current mode. It also handles
force feedback for the haptic device. Switching is handled with a serviceClient instead of several flag topics.
*/

geometry_msgs::Twist plan_point;
geometry_msgs::Twist haptic_point;
geometry_msgs::Twist robot_point;
geometry_msgs::Twist final_point;
bool left_pressed;
bool middle_pressed;
bool right_pressed;
bool held = false;
bool plan_received = false;
bool haptic_received = false;
bool pedal_received = false;
bool h_pose_received = false;
bool robot_received = false;
bool timer_white = false;
bool timer_grey = false;
bool update_flag;
bool white_press;
bool grey_press;
int click_count = 0;
int right_click_count = 0;

std_msgs::Bool intensity_flag;
bool got_intensity = false;

// simulation box
float x_max = 1;
float x_min = -1;
float y_max = 1;
float y_min = -2;
float z_max = 1;
float z_min = 0;

void pedal_callback(const surgery_sim::PedalEvent &  _data){
	// read the pedal input. Only reads unput when pressed once, so if held, it will not continue reading input.
	if (_data.left_pedal == 1){
		if (!held){
			left_pressed = true;
		} else{
			left_pressed = false;
		}
		held = true;
	} else if (_data.middle_pedal == 1){
		if (!held){
			middle_pressed = true;
		} else{
			middle_pressed = false;
		}
		held = true;
	} else if (_data.right_pedal == 1){
		if (!held){
			right_pressed = true;
			right_click_count++;
		} else{
			right_pressed = false;
		}
		held = true;
	} else{
		held = false;
		left_pressed = false;
		middle_pressed = false;
		right_pressed = false;
	}
	pedal_received = true;
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

void stop_callback(const std_msgs::Bool & _data){
	intensity_flag = _data;
	got_intensity = true;
}

// method for checking if target position is within the bounding box. Sets position to bounding box values if too large
void check_box(const geometry_msgs::Twist point){
	if ((point.linear.x <= x_max) && (point.linear.x >= x_min)) {
		final_point.linear.x = point.linear.x;
	} else if (point.linear.x < x_min){
		final_point.linear.x = x_min;
	} else{
		final_point.linear.x = x_max;
	}

	if ((point.linear.y <= y_max) && (point.linear.y >= y_min)) {
		final_point.linear.y = point.linear.y;
	} else if (point.linear.y < y_min){
		final_point.linear.y = y_min;
	} else{
		final_point.linear.y = y_max;
	}

	if ((point.linear.z <= z_max) && (point.linear.z >= z_min)) {
		final_point.linear.z = point.linear.z;
	} else if (point.linear.z < z_min){
		final_point.linear.z = z_min;
	} else{
		final_point.linear.z = z_max;
	}

	final_point.angular.x = point.angular.x;
	final_point.angular.y = point.angular.y;
	final_point.angular.z = point.angular.z;
}

// This is for remebering the orignial position at lines 268-270 to avoid kicks
double origin_x = -0.007;
double origin_y = -.02;
double origin_z = 0.04;

// desired values of x, y, z for centering the haptic device
double x_d = origin_x;
double y_d = origin_y;
double z_d = origin_z;

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
omni_msgs::OmniFeedback centering_force_reset;

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
	double tau = .3;
	centering_force.force.x = (kp_x*e_x + kd_x*de_x)*(1-tau) + tau*centering_force_prev.force.x;
	centering_force.force.y = (kp_y*e_y + kd_y*de_y)*(1-tau) + tau*centering_force_prev.force.y;
  centering_force.force.z = (kp_z*e_z + kd_z*de_z)*(1-tau) + tau*centering_force_prev.force.z + 0.2;//the last component is for the effect hand weight
	// reduce the kicks by resetting the force feedbacks when the reference changes
 	e_x_prev = e_x;
	e_y_prev = e_y;
	e_z_prev = e_z;
	centering_force_prev = centering_force;
}

// This method was originally created for waiting while the arm retracts when swapping from autonomous to manual
// Might help reduce unwanted kicks at switch. Don't know if without timer, there won't be kicks. There is a chance
// I coded it well enough to switch without the need of a timer.
void timer_callback(const ros::TimerEvent& event){
	if (white_press){
		timer_white = true;
	} else{
		timer_grey = true;
	}
}

void update_force(double x, double y, double z){
	x_d = x;
	y_d = y;
	z_d = z;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "switch_node");
  ros::NodeHandle node;

	ros::NodeHandle home("~");
  bool sim = true;
	home.getParam("sim", sim); // options are: "true"; "false"
  
  // initializing server interaction
  ros::ServiceClient traj_client = node.serviceClient<surgery_sim::Reset>("reset");
  ros::ServiceClient haptic_client = node.serviceClient<surgery_sim::Reset>("haptic_server");
  ros::ServiceClient frame_client = node.serviceClient<surgery_sim::Reset>("frame_server");
  ros::ServiceClient plan_client = node.serviceClient<surgery_sim::Reset>("plan_server");
  ros::ServiceClient overlay_client = node.serviceClient<surgery_sim::Reset>("overlay_server");
  surgery_sim::Reset reset;
  
	// timer creation
  ros::Timer timer = node.createTimer(ros::Duration(1), timer_callback, true);
 
 	// subscriber for reading confidence intensity flag
	ros::Subscriber stop_sub = node.subscribe("/stop_auto", 1, stop_callback);
	// subscriber for reading haptic device pedal input
	ros::Subscriber pedal_sub = node.subscribe("/pedal", 1, pedal_callback);
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
	ros::Publisher pub_mode= node.advertise<std_msgs::Int32>( "/current_mode", 1 );
  
  bool white_flag = true;
  bool grey_flag = false;
	bool stop = false;
  bool robot_flag = true;
	std_msgs::Int32 current_mode;
	current_mode.data = 1; // 0 = autonomous; 1 = manual
  
  double rx;
	double ry;
	double rz;
  
  geometry_msgs::Twist robot_initial;

	if (!sim){
		// real robot box
		x_max = 0.11;
		x_min = 0.03028;
		y_max = 0.0008;
		y_min = -0.669;
		z_max = 0.0762;
		z_min = 0.018;
	}
	// x=0.03028, y=0.0008, z=0.0181
  
  ros::Rate rate(10.0);
  while (node.ok()){
  	if (robot_received){
  		if(robot_flag){
				robot_initial = robot_point;
				robot_flag = false;
			}else{
				// While in manual mode, the force updates to where the pen already is, essentially makes it so no force exists
				// unless the user moves faster than the loop frequency
				// click count flag is for when the robot is first initialized
				if (((!white_flag) || (click_count == 0)) && (right_click_count > 0)){
					rx = 2 * (robot_point.linear.x - robot_initial.linear.x) + origin_x;
					ry = 2 * (robot_point.linear.y - robot_initial.linear.y) + origin_y;
					rz = 2 * (robot_point.linear.z - robot_initial.linear.z) + origin_z;
					
				}else{
					rx = phantom_pos.pose.position.x;
					ry = phantom_pos.pose.position.y;
					rz = phantom_pos.pose.position.z;
				}
				
			}
  	}
  	
  	// white flag for autonomous
		// grey falg for manual
		// might have been a little excessive with flags, but then again, maybe it was the perfect amount
  	if (pedal_received){
			/*
			The experiment will not start until the left most pedal is pressed. In the meantime, the mode that the 
			experiment starts with can be determined with the middle pedal. Could also make this an rqt_reconfigure thing
			or a argument from the launch file
			click_count is raised by the press of the left pedal
			*/
			if ((middle_pressed) && (click_count == 0)){
				reset.request.preview = true;
				if (grey_flag){
					reset.request.manual = false;
					white_flag = true;
					grey_flag = false;
					current_mode.data = 0;
				} else{
					reset.request.manual = true;
					grey_flag = true;
					white_flag = false;
					current_mode.data = 1;
				}
				overlay_client.call(reset);
			}

			/*
			when a the left pedal is pressed, it flips the white and grey flags, and sends service calls to the appropriate
			nodes with the updated flags
			*/
  		if (left_pressed && white_flag){
  			white_flag = false;
  			grey_flag = true;
  			white_press = true;
  			grey_press = false;
  			timer_grey = false;
  			reset.request.plan_start = true;
  			reset.request.plan_flag = true;
  			reset.request.hap_start = false;
  			reset.request.hap_flag = false;
				reset.request.preview = false;
				if (!stop){
					overlay_client.call(reset);
				}
  			haptic_client.call(reset);
  			frame_client.call(reset);
  			if (click_count > 0){
  				plan_client.call(reset);
  			} else{
  				traj_client.call(reset);
  			}
    		ROS_INFO("out: started plan. haptic off");
				current_mode.data = 0;
    		haptic_received = false;
    		click_count ++;
    		timer.stop();
    		timer.setPeriod(ros::Duration(.1));
    		timer.start();
  		}
  		else if (left_pressed && grey_flag){
  			white_flag = true;
  			grey_flag = false;
  			white_press = false;
  			grey_press = true;
  			timer_white = false;
  			reset.request.plan_flag = false;
  			reset.request.plan_start = false;
  			reset.request.hap_start = true;
  			reset.request.hap_flag = true;
				reset.request.preview = false;
				if (!stop){
					overlay_client.call(reset);
				}
  			traj_client.call(reset);
  			plan_client.call(reset);
  			haptic_client.call(reset);
  			//frame_client.call(reset);
    		ROS_INFO("out: resetting plan. haptic on");
				current_mode.data = 1;
    		plan_received = false;
    		click_count ++;
    		timer.stop();
    		timer.setPeriod(ros::Duration(.1));
    		timer.start(); 		
  		} else if ((right_pressed) && (right_click_count > 2)){
				stop = true;
  			reset.request.plan_start = false;
  			reset.request.hap_start = false;
				overlay_client.call(reset);
			}
  			
			// once the node has received points from the correct topics, and the correct flags are flipped, it 
			// publishes to the ur5e_controller
  		if (timer_white && plan_received && !stop && (click_count > 0)){
				check_box(plan_point);
				if (!intensity_flag.data){
					pub_robot.publish(final_point);
				}
			} else if (timer_grey && haptic_received && !stop && (click_count > 0)){
				check_box(haptic_point);
				pub_robot.publish(final_point);
			}
		}

		// publishes force
		if (h_pose_received && robot_received & !stop){
			update_force(rx, ry, rz);
			calc_center_force();
			force_pub.publish(centering_force);
		}
		
		if (stop){
			centering_force.force.x = 0;
			centering_force.force.y = 0;
			centering_force.force.z = 0;
			force_pub.publish(centering_force);
		}

		pub_mode.publish(current_mode);
		
		rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
