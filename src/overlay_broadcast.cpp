#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <surgery_sim/Reset.h>


/*
	TODO remove adding a frame to the plan to work again. The commented out code shuold all be fine, what isnt was for demonstration. Try to go back and not make an extra frame, or see if you can make a transform without broadcasting. To get the pointcloud to work, I think change the frame id, or do some frame transformation like in his code on git.
	
	What I had originally was fine, except creating a broadcaster and using the wrong frame for the header of the debug pcl. I also didn't double check the plan (was modifying x when I should have y).
*/

pcl::PointCloud<pcl::PointXYZI> dbg_cloud;


void point_callback(const geometry_msgs::PointStamped & _data){
	pcl::PointXYZI tmp;
	tmp.x = _data.point.x;
	tmp.y = _data.point.y;
	tmp.z = _data.point.z;
	dbg_cloud.push_back(tmp);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "overlay_broadcast");

  ros::NodeHandle node;
  
	ros::Subscriber point_sub = node.subscribe("/clicked_point", 1, point_callback);
  ros::Publisher dbg_traj_pub = node.advertise<pcl::PointCloud<pcl::PointXYZI> > ("test_cloud",1);

  tf::TransformListener listener;
  
  // Initiating variables
  // pcl::PointXYZI tmp;
  // tmp.x = 0.08436;
	// tmp.y = -0.6039;
	// tmp.z = 0.07457;
	// dbg_cloud.points.push_back(tmp);

	// tmp.x = 0.0389245;
	// tmp.y = -0.5374;
	// tmp.z = 0.0291;
	// dbg_cloud.points.push_back(tmp);

	// tmp.x = 0.0215;
	// tmp.y = -0.61486;
	// tmp.z = 0.0154;
	// dbg_cloud.points.push_back(tmp);

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);

  while (node.ok()){
  	std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = std::string("world");
		dbg_cloud.header = pcl_conversions::toPCL(header);
		dbg_traj_pub.publish(dbg_cloud);
				
		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;
};
