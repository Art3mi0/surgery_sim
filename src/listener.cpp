#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Reset.h>

geometry_msgs::Twist robot_initial;
bool robot_received = false;
bool robot_record = false;
bool frame_record = false;
bool publish = false;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_initial = _data;
	robot_received = true;
}

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  if (req.hap_start){
  	frame_record = true;
  	publish = false;
  	ROS_INFO("request: Initialize Haptic");
  } else{
  	publish = false;
  } /*
	if (req.flag){
		reset_traj = false;
		control_mode = 1;
		ROS_INFO("request: do not reset");
	} else{
		control_mode = 0;
		reset_traj = true;
		ROS_INFO("request: reset");
	} */
  //ROS_INFO("request: flag=%s", (bool)req.flag ? "true" : "false");
  //ROS_INFO("sending back response: [%s]", (bool)res.out ? "true" : "false");
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  
  // initializing server
	ros::ServiceServer service = node.advertiseService("haptic_server", flag);

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
	// Publisher for the kinematic solver. Skips the plan point generation
  ros::Publisher pub_robot = node.advertise<geometry_msgs::Twist>("/refhap", 1);
  // Publisher for the RViz visual
  ros::Publisher pub_point= node.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );

  tf::TransformListener listener;
  
  // Initiating variables
  geometry_msgs::Twist initial;
  //geometry_msgs::PointStamped point;
  int seq = 0;
  float x;
  float y;
  float z;

  int loop_freq = 10;
  float dt = (float) 1/loop_freq;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("base", "haptic",  
                               ros::Time(0), transform);
      
      if (frame_record){
				x = transform.getOrigin().x();
				y = transform.getOrigin().y();
				z = transform.getOrigin().z();
				if (x != 0){
					frame_record = false;
					robot_record = true;
				}
			
			}
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
		
		// Records the initial position of the robot once
		if (robot_received && robot_record){
			initial = robot_initial;
			robot_record = false;
			publish = true;
			//std::cout << initial;
		}
		
		// Records the initial position of the pen
		// When troubleshooting, the first reading was 0, but the next one should be an accurate reading
		
		if (robot_received && publish && !frame_record){		
			// If the point is not re-created each loop, there will be a bug where the robot keeps moving when you do not want it to
			geometry_msgs::Twist rob_point;
			
			// The leading int is the scalar. The larger the number, the more the robot will move
			// The initial pen position is saved and subtracted from the current, so new movements of the pen will move the robot accordingly by being added to the robot's starting position which is saved. This is to allow the user to choose a comfortable starting position for the pen in the real world	
	    rob_point.linear.x = 2.6 *(transform.getOrigin().x() - x) + initial.linear.x;
	    rob_point.linear.y = 2.6 *(transform.getOrigin().y() - y) + initial.linear.y;
	    rob_point.linear.z = 3.5 *(transform.getOrigin().z() - z) + initial.linear.z;
	    rob_point.angular.x = initial.angular.x;
	    rob_point.angular.y = initial.angular.y;
	    rob_point.angular.z = initial.angular.z;
	    pub_robot.publish(rob_point);
	  
		}

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
