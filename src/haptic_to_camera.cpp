#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>


geometry_msgs::PointStamped point_data;
geometry_msgs::Point init;

bool point_received = false;
bool init_stored = false;


void get_pose(const geometry_msgs::PoseStamped &  _data){
	// read the pose of the haptic device
	pose_data = _data;

	point_received = true;
}


int main(int argc, char * argv[]){
	ros::init(argc,argv,"surgery_sim");
	ros::NodeHandle nh_;
	// subscriber for reading the haptid device pose
	ros::Subscriber haptic_sub = nh_.subscribe("/phantom/phantom/pose", 1, get_pose);
	
	// publisher for writing the haptic device point
	ros::Publisher pub_point= nh_.advertise<geometry_msgs::PointStamped>( "/haptic_point", 1 );
	
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	tf::TransformListener listener(ros::Duration(10));
	
	int seq = 0;
	
	// define a pointstamp for displaying the point
	geometry_msgs::PointStamped point;
	
	while(ros::ok()){
		if (point_received){				
			if (!init_stored){
			init = pose_data.pose.position;
			init_stored = true;
			}
			
			//apply a header to the point and publish it
			std_msgs::Header header;
			header.stamp = ros::Time::now();
			header.seq = seq++;
			header.frame_id = std::string( "tool_end" );
			point.header = header;
			point.point = pose_data.pose.position;
			point.point.x -= init.x;
			point.point.y -= init.y;
			point.point.z -= init.z - .03;
			try{
  				geometry_msgs::PointStamped base_point;
      				listener.transformPoint("camera_link", point, t_point);
      				pub_point.publish(t_point);
     			}
     			catch(tf::TransformException& ex){
       			ROS_ERROR("Received an exception trying to transform a point from \"tool_end\" to \"camera_link\": %s", ex.what());
     			}
			
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;	

}
