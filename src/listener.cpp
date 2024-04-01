#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Reset.h>

geometry_msgs::Twist robot_current;
bool robot_received = false;
bool robot_record = false;
bool frame_record = false;
bool publish = false;
bool retract = false;
float retract_z = 0.0; // 0ld - .02

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_current = _data;
	robot_received = true;
}

bool flag(surgery_sim::Reset::Request  &req,
         surgery_sim::Reset::Response &res){
  if (req.hap_start){
  	frame_record = true;
  	publish = false;
  	retract = true;
  	ROS_INFO("request: Initialize Haptic");
  	ROS_INFO("Retracting Arm..");
  } else{
  	publish = false;
  } 
  return true;
}


int sgn(double v) {
  if (v < 0) return -1;
  if (v > 0) return 1;
  return 0;
}

double dead_zone(double _val){
	double d = 0.002; //deadzone value of 1cm
	if (fabs(_val) <= d){
		return 0.0;
	}else{
		return _val - sgn(_val)*d;  
	}
	
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
	ros::Publisher pub_test_hap = node.advertise<geometry_msgs::Twist>("/hap_test", 1);

  tf::TransformListener listener;
  
  // Initiating variables
  geometry_msgs::Twist initial;
  //geometry_msgs::PointStamped point;
  int seq = 0;
  float x;
  float y;
  float z;
  float tran_x;
  float tran_y;
  float tran_z;
  float retract_z;

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
			initial = robot_current;
			robot_record = false;
			publish = true;
			//std::cout << initial;
		}
		
		// Records the initial position of the pen
		// When troubleshooting, the first reading was 0, but the next one should be an accurate reading
		
		if (robot_received && publish && !frame_record){		
			// If the point is not re-created each loop, there will be a bug where the robot keeps moving when you do not want it to
			geometry_msgs::Twist rob_point;
			geometry_msgs::Twist hap_point;
			tran_x = transform.getOrigin().x() - x;
			tran_y = transform.getOrigin().y() - y;
			tran_z = transform.getOrigin().z() - z;

			hap_point.linear.x = tran_x;
			hap_point.linear.y = tran_y;
			hap_point.linear.z = tran_z;
			
			// The leading int is the scalar. The larger the number, the more the robot will move
			// The initial pen position is saved and subtracted from the current, so new movements of 
			// the pen will move the robot accordingly by being added to the robot's starting position 
			// which is saved. This is to allow the user to choose a comfortable starting position for 
			// the pen in the real world	
			rob_point.linear.x = 2 *dead_zone(tran_x) + initial.linear.x;
			rob_point.linear.y = 2 *dead_zone(tran_y) + initial.linear.y;
			rob_point.linear.z = 2 *dead_zone(tran_z) + initial.linear.z + retract_z;
			rob_point.angular.x = initial.angular.x;
			rob_point.angular.y = initial.angular.y;
			rob_point.angular.z = initial.angular.z;
			pub_test_hap.publish(hap_point);
	    
			// if (retract){
			// 	retract_z = (.5* ((initial.linear.z + retract_z) - robot_current.linear.z)) + robot_current.linear.z;
			// 	rob_point.linear.z = retract_z;
			// 	if (robot_current.linear.z >= initial.linear.z + (retract_z - .01)){
			// 		retract = false;
			// 		ROS_INFO("Retracting Complete");
			// 	}
			// }
			
			pub_robot.publish(rob_point);
	  
		}

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
