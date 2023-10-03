#include "ros/ros.h"
#include "surgery_sim/Reset.h"

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
	if (req.flag){
		res.out = true;
	} else{
		res.out = false;
	}
  ROS_INFO("request: flag=%s", (bool)req.flag ? "true" : "false");
  ROS_INFO("sending back response: [%s]", (bool)res.out ? "true" : "false");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("reset", flag);
  ROS_INFO("Ready to output flag.");
  ros::spin();

  return 0;
}
