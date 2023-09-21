#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

geometry_msgs::Twist robot_initial;
bool robot_received = false;

void robot_callback(const geometry_msgs::Twist &  _data){
	// Read the pose of the robot
	robot_initial = _data;
	robot_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

	// Subscriber for reading the robot pose
	ros::Subscriber robot_sub = node.subscribe("/ur5e/toolpose", 1, robot_callback);
	// Publisher for the kinematic solver. Skips the plan point generation
  ros::Publisher pub_robot = node.advertise<geometry_msgs::Twist>("/reftraj", 1);
  // Publisher for the RViz visual
  ros::Publisher pub_point= node.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );

  tf::TransformListener listener;
  
  // Initiating variables
  bool init_saved = false;
  bool robot_recorded = false;
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
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
		
		// Records the initial position of the robot once
		if (robot_received && !robot_recorded){
			initial = robot_initial;
			robot_recorded = true;
			//std::cout << initial;
		}
		
		// Records the initial position of the pen
		// When troubleshooting, the first reading was 0, but the next one should be an accurate reading
		if (!init_saved){
			x = transform.getOrigin().x();
			y = transform.getOrigin().y();
			z = transform.getOrigin().z();
			if (x != 0){
				init_saved = true;
			}
			
			//std::cout << x;
			//std::cout << y;
			//std::cout << z;
			
		}
		
		if (init_saved){
		// Top portion for visualizing in RViz, bottom portion for moving the simulated robot
		// Top portion may not be updated to account changes from the listener
		/*
			geometry_msgs::PointStamped point;
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "base_link" );
			point.header = header;
			// The two values being subtracted are to show correctly in the pointstamped
			// I think the real robot should have the values added
			point.point.x = transform.getOrigin().x() - x - initial.linear.x;
			point.point.y = transform.getOrigin().y() - y - initial.linear.y;
			point.point.z = transform.getOrigin().z() - z + initial.linear.z;
			pub_point.publish(point);
		*/
		
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
