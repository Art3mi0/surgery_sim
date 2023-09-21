#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>


// Didn't transform point according to frame. Instead checked rviz for axis orientation
// Add subscriber to button input and have the second button reset position or have the second button pause the data to allow user to move pen elsewhere and it still stays at tooltip but can move further
// Might need to add subscriber to robot pose for aiding in resetting position not sure though
// Point should be in the camera frame and converted to the robot frame (might want to clean up model and remove extra tool frame and make sure optical frame is properly set)
// For home, just make random point moving in one direction by starting at 000 and adding x by .1 until x is greater than 1.5 or something then subtract until x is 0 and repeat then figure out transformations 

geometry_msgs::PoseStamped pose_data;
geometry_msgs::Point init;

bool point_received = false;
bool init_stored = false;
bool frames_found = false;


void get_pose(const geometry_msgs::PoseStamped &  _data){
	// read the pose of the haptic device
	pose_data = _data;

	point_received = true;
}



int main(int argc, char * argv[]){
	ros::init(argc,argv,"haptic_visualize");
	ros::NodeHandle nh_;
	// subscriber for reading the haptid device pose
	ros::Subscriber haptic_sub = nh_.subscribe("/phantom/phantom/pose", 1, get_pose);
	
	// publisher for writing the haptic device point
	ros::Publisher pub_point= nh_.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );
	ros::Publisher pub_tran= nh_.advertise<geometry_msgs::TransformStamped>( "/transformation", 1 );
	
	int loop_freq = 1;
	ros::Rate loop_rate(loop_freq);
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	
	int seq = 0;
	
	// define a pointstamp for displaying the point
	geometry_msgs::PointStamped point;
	
	while(ros::ok()){
		geometry_msgs::TransformStamped transformStamped;
		try{
			transformStamped = tfBuffer.lookupTransform("tool_end", "camera_link",
			ros::Time(0));
			pub_tran.publish(transformStamped);
			if (!frames_found){
			std::cout << "frames found";
			frames_found = true;
			}
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		if (point_received){				
			if (!init_stored){
			init = pose_data.pose.position;
			init_stored = true;
			}
			
			// apply a header to the point and publish it
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "tool_end" );
			point.header = header;
			point.point = pose_data.pose.position;
			point.point.x -= init.x;
			point.point.x = -1 * point.point.x;
			point.point.y -= init.y;
			point.point.z -= init.z + .03;
			point.point.z = -1 * point.point.z;
			pub_point.publish(point);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	

}
