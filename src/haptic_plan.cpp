#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <surgery_sim/Plan.h>
#include <geometry_msgs/PointStamped.h>
#include <omni_msgs/OmniButtonEvent.h>


geometry_msgs::Twist robot_initial;
omni_msgs::OmniButtonEvent button_data;
bool robot_received = false;
bool plan_created = false;
bool button_received = false;


void robot_callback(const geometry_msgs::Twist &  _data){
	// read the pose of the robot
	robot_initial = _data;
	robot_received = true;
}

void button_callback(const omni_msgs::OmniButtonEvent &  _data){
	// read the pose of the robot
	button_data = _data;
	button_received = true;
}

		
int main(int argc, char * argv[]){
	ros::init(argc,argv,"haptic_plan");
	ros::NodeHandle nh_;
	// subscriber for reading the robot pose
	ros::Subscriber robot_sub = nh_.subscribe("/ur5e/toolpose", 1, robot_callback);
	// subscriber for reading haptic device button input
	ros::Subscriber button_sub = nh_.subscribe("/phantom/phantom/button", 1, button_callback);
	
	// publisher for writing the plan
	ros::Publisher pub_plan= nh_.advertise<surgery_sim::Plan>( "/plan", 1);
	
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	
	surgery_sim::Plan plan;
	geometry_msgs::Twist plan_point;
	geometry_msgs::Twist initial;
	std::vector<geometry_msgs::Twist> plan_points;
	bool button_pushed = false;
	bool robot_recorded = false;
	bool started_recording = false;
	bool plan_created = false;
	bool printed = false;
	bool recorded = false;
	int count = 0;
	
	int seq = 0;
	
	while(ros::ok()){
		if (robot_received && !robot_recorded){
			initial = robot_initial;
			robot_recorded = true;
			plan_points.push_back(initial);
			plan_point.linear.x = initial.linear.x + .1;
			plan_point.linear.y = initial.linear.y;
			plan_point.linear.z = initial.linear.z;
			plan_point.angular.x = initial.angular.x;
			plan_point.angular.y = initial.angular.y;
			plan_point.angular.z = initial.angular.z;
			std::cout << "Robot initial pose recorded";
			plan_points.push_back(plan_point);
			plan.points = plan_points;
		}
		
		if (plan_created){
			pub_plan.publish(plan);
		}

		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	

}
