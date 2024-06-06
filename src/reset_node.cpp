#include <ros/ros.h>
#include <surgery_sim/PedalEvent.h>
#include <std_msgs/Bool.h>


surgery_sim::PedalEvent pedal_data;
bool pedal_received = false;

void pedal_callback(const surgery_sim::PedalEvent &  _data){
	// read the pedal input
	pedal_data = _data;
	pedal_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "reset_node");

  ros::NodeHandle node;

	// subscriber for reading haptic device pedal input
	ros::Subscriber pedal_sub = node.subscribe("/pedal", 1, pedal_callback);

	// publisher for initialize flag
	ros::Publisher init_pub= node.advertise<std_msgs::Bool>( "/init_flag", 1);
	// publisher for start flag
	ros::Publisher start_pub= node.advertise<std_msgs::Bool>( "/start_flag", 1);

	std_msgs::Bool init;
	std_msgs::Bool start;
	init.data = false;
	start.data = false;
	int mode = 0;
	int ctr = 0;
  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){


	// Goal is to make user initialize, stop initialization, then swap mode
	// Currently, they can just cancel initialization and start or initialize and start while constantly initializing
	// Solution : make signal for when both left and middle pedals have been pressed and make init block care
	if (pedal_received){
		if ((pedal_data.left_pedal == 1) || (pedal_data.middle_pedal == 1)){
			ctr ++;
		}
		// initializing block
		if (((mode % 2) == 0) && (pedal_data.right_pedal == 1) && (ctr > 0)){
			init.data = true;
			start.data = false;
			ctr = 0;
			mode++;
		
		// Starting block
		} else if (((mode % 2) == 1) && (pedal_data.right_pedal == 1) && (ctr > 0)){
			init.data = false;
			start.data = true;
			ctr = 0;
			mode++;
		}
	}
	
	init_pub.publish(init);
	start_pub.publish(start);

	loop_rate.sleep();
	ros::spinOnce();
  }
  return 0;
};