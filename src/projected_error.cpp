#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/Float32.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <surgery_sim/Density.h>
#include <list>
#include <surgery_sim/PedalEvent.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> //rsd - was giving error on swami
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

list<float>::iterator it;
surgery_sim::Density density;
pcl::PointCloud<pcl::PointXYZ> plan_cloud;
pcl::PointCloud<pcl::PointXYZ> t_plan_cloud;
pcl::PointCloud<pcl::PointXYZ> user_plan_cloud;
pcl::PointCloud<pcl::PointXYZ> t_user_plan_cloud;
pcl::PointCloud<pcl::PointXYZ> selected_cloud;
pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
bool pcl_received = false;
bool user_pcl_received = false;
bool cropped_received = false;
bool selected_received = false;

sensor_msgs::PointCloud2 test;
int edge1_start = 99;
int edge2_start = 99;
int edge3_start = 99;
int edge4_start = 99;

float auto_to_man_cost = 3.0; // 2.0 +- 1.0
float man_to_auto_cost = 2.1; // 1.7 +- 0.4
float man_cost = 3.59; // 2.49 +- 1.1

float confidence_threshold = 8.4241; //original
float auto_to_man_threshold = 5.59; // cost to switch to manual
float man_to_auto_threshold = 3.12; // just the cost of teleoperation
float pitch = -2.349;
float roll = 0.038;

int right_count = 0;
bool held = false;

// cross product of two 3D vectors
geometry_msgs::Vector3 cross_product(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;

  c.x = a_.y*b_.z - a_.z*b_.y;
  c.y = a_.z*b_.x - a_.x*b_.z;
  c.z = a_.x*b_.y - a_.y*b_.x;

  return c;
}

// subtraction of two 3D vectors A-B
geometry_msgs::Vector3 subt(geometry_msgs::Vector3 & a_ , geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;
  c.x = a_.x - b_.x;
  c.y = a_.y - b_.y;
  c.z = a_.z - b_.z;

  return c;
}

float norm(geometry_msgs::Vector3 & a_){

  return sqrt(a_.x*a_.x + a_.y*a_.y + a_.z*a_.z);

}

float mid_point_dist(geometry_msgs::Vector3 & a_, geometry_msgs::Vector3 & b_){
	geometry_msgs::Vector3 c;
	c.x = (a_.x + b_.x)/2;
	c.y = (a_.y + b_.y)/2;
	c.z = (a_.z + b_.z)/2;
	return norm(c);
}

// finding the vector length between points A and B
float vec_len(geometry_msgs::Vector3 & a_ , geometry_msgs::Vector3 & b_){

  geometry_msgs::Vector3 c;
  c.x = a_.x - b_.x;
  c.y = a_.y - b_.y;
  c.z = a_.z - b_.z;

  return norm(c);
}

// distance of a point x0 from the line formed by x1 and x2
float dist_from_line(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2){

  geometry_msgs::Vector3 diff1;  
  geometry_msgs::Vector3 diff2;
  geometry_msgs::Vector3 diff3;

  diff1 = subt(x0, x1);  
  diff2 = subt(x0, x2);  
  diff3 = subt(x2, x1);  

  geometry_msgs::Vector3 cross1;
  
  cross1 = cross_product(diff1, diff2);
  


  float d = 0.0;
  d = norm(cross1)/norm(diff3);
  return d;

}

// check if the point x0 is between the cylinder caps formed around the line from x1 to x2 
bool between_caps(geometry_msgs::Vector3 x0, geometry_msgs::Vector3 x1,geometry_msgs::Vector3 x2, float h){
  geometry_msgs::Vector3 diff1;  
  geometry_msgs::Vector3 diff2;
  
  diff1 = subt(x0, x1);  
  diff2 = subt(x2, x1);  

  float dot_prod = diff1.x*diff2.x + diff1.y*diff2.y + diff1.z*diff2.z;
  
  if(dot_prod < 0.0 || dot_prod > h)
	return false;
  else
	return true;

}


// finds the number of points in the point cloud that fit in a cylinder between a two marker positions (i.e. start and end points)
float calc_density(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, pcl::PointCloud<pcl::PointXYZ> pcd){
	pcl::PointCloud<pcl::PointXYZ>::iterator pi=pcd.begin();
	float density = 0.0;
	int ctr = 0;
  float r_c = 0.0025;//radius of the cylinder for finding point dentisty between two points
	double h = vec_len(x1, x2);
	//cout <<"read" << pcd.points.size()<< " points from the pcd"<<endl;
	for( ; pi!=pcd.end(); pi++ ) {

		geometry_msgs::Vector3 x0;
		x0.x = pi->x;
		x0.y = pi->y;
		x0.z = pi->z;
	
		float distance = 0.0;
		distance = dist_from_line(x0, x1, x2);
		
		bool inside_caps = between_caps(x0, x1, x2,h);
		if (distance <= r_c && inside_caps)
			density += 1.0;

		if (ctr == 0){
			 	//cout <<"start point (cap) " <<  x1<<endl;
			 	//cout <<"end point (cap) " <<  x2<<endl;

				//cout <<"test point distance from the line " << distance<< " meters ---- height is: " <<h<<" meters"<<endl;

		}
 		ctr ++;
	}


 	 double cyl_volume = M_PI*r_c*r_c*h*1000000000;//volume of the cylinder formed by x1-x2 line and radius r_c
 // cout << "volume was: " << cyl_volume<< " for h= "<< h << endl; 
	//return density/pcd.points.size();
	return density/cyl_volume;
}

void pedal_callback(const surgery_sim::PedalEvent & _data){
	if (_data.right_pedal == 1){
			if (!held){
					held = true;
					right_count++;
			}
	} else{
			held = false;
	}
}

float calc_projected_err(float roll_, float pitch_, float dist_, float density_){
	float w0 = 1.0;//0.65; //for overall normalization based on observed errors	
	float w1 = 1.0*0;//0.25;//weight for roll
	float w2 = 1.0*0;//0.25;//weight for pitch
	float w3 = 1.0*0;//0.5;//weight for dist
	float w4 = 1.0;//weight for density

	float e1 = 0.000065*roll_*roll_ + 0.0046*roll_ + 1.2134;
	float e2 = 0.0001564*pitch_*pitch_ - 0.00016*pitch_ + 1.1737;
	float e3 = 0.0761*(dist_*100-11) - 1.3919;//converted to cm first
	float e4 =  10.5458*exp(-0.8991*density_);

	//std::cout<<" e1: "<< e1 <<" e2: "<< e2 <<" e3: "<< e3 <<" e4: "<< e4 <<std::endl; 
	return w0*(w1*e1+w2*e2+w3*e3+w4*e4);
}

bool check_edge(geometry_msgs::Vector3 point, pcl::PointCloud<pcl::PointXYZ> square){
	bool flag = false;

	for (int i = 0; i < square.size(); i ++){
		if ((point.x == square.points[i].x) && (point.y == square.points[i].y)){
			flag = true;
		}
	}

	return flag;
}


void user_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, user_plan_cloud);
	if (user_plan_cloud.size() > 1){
		user_pcl_received = true;
	}
}

void pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, plan_cloud);
	pcl_received = true;
}

void selected_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	pcl::fromROSMsg(_data, selected_cloud);
	selected_received = true;
}

void cropped_pcl_callback(const sensor_msgs::PointCloud2 &  _data){
	test = _data;
	test.fields[3].name = "intensity";
	pcl::fromROSMsg(_data, cropped_cloud);
	cropped_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "projected_error");
  ros::NodeHandle node;
	ros::NodeHandle home("~");
	
	bool edge_only = true;
	std::string start_mode = "manual";
	home.getParam("edge_only", edge_only); //
	home.getParam("start_mode", start_mode); // options are : manual; auto
	home.getParam("confidence_threshold", confidence_threshold); // options are : manual; auto

	ros::Subscriber pedal_sub = node.subscribe("/pedal", 1, pedal_callback);
	ros::Subscriber user_plan_cloud_sub = node.subscribe("/user_plancloud", 1, user_pcl_callback);
	ros::Subscriber plan_cloud_sub = node.subscribe("/plancloud", 1, pcl_callback);
	ros::Subscriber crop_cloud_sub = node.subscribe("/cropped_pointcloud", 1, cropped_pcl_callback);
	ros::Subscriber selected_cloud_sub = node.subscribe("/selected_points", 1, selected_pcl_callback);
	//ros::Subscriber crop_cloud_sub = node.subscribe("/camera/depth_registered/points", 1, cropped_pcl_callback);

	ros::Publisher pub_path_confidence= node.advertise<sensor_msgs::PointCloud2>("/path_confidence", 10);

  tf::TransformListener listener;
	bool transformed = false;
	std_msgs::Float32 den_avg32;
	int remove_start;
	bool once = true;
	bool edge_found = false;

	float cost_of_next;
	float cost_of_current;
	float t_cost;


	geometry_msgs::Vector3 x0;
  geometry_msgs::Vector3 x1;
  geometry_msgs::Vector3 x2;
  

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
		if ((user_pcl_received) && (pcl_received)){
			try
				{
					if (!edge_only){
						pcl_ros::transformPointCloud("camera_color_optical_frame", user_plan_cloud, t_user_plan_cloud, listener);
					} else{
						pcl_ros::transformPointCloud("camera_color_optical_frame", user_plan_cloud, t_plan_cloud, listener);
						pcl_ros::transformPointCloud("camera_color_optical_frame", plan_cloud, t_user_plan_cloud, listener);
					}
					transformed = true;
				}
			catch (tf::TransformException ex)
				{
					ROS_WARN("%s", ex.what());
					ros::Duration(1.0).sleep();
				}
		}

		if (transformed){
			pcl::PointCloud<pcl::PointXYZI> confidence_path_pcl;
			pcl::PointCloud<pcl::PointXYZI> edge_pcl;
			int ctr = 0;
			bool man_flag = false;
			bool hover_init = false;
			int pts_len = t_user_plan_cloud.points.size();
			confidence_threshold = auto_to_man_threshold;
			bool auto_mode = true;

			int edge_count = 0;
			float current_edge_x;
			float current_edge_y;

			t_cost = auto_to_man_cost;
			cost_of_next = man_cost;

			if(pts_len == 0 || pts_len == 1){
				ROS_INFO("waiting for the pcl points (min = 2) to be received");
			}else{
				for (int pts_id =0; pts_id < pts_len; pts_id++){
					int start_pt = pts_id % pts_len;
					int end_pt = (pts_id +1 )% pts_len;

					x1.x = t_user_plan_cloud.points[start_pt].x;
					x1.y = t_user_plan_cloud.points[start_pt].y;
					x1.z = t_user_plan_cloud.points[start_pt].z;

					x2.x = t_user_plan_cloud.points[end_pt].x;
					x2.y = t_user_plan_cloud.points[end_pt].y;
					x2.z = t_user_plan_cloud.points[end_pt].z;

					float density = calc_density(x1, x2, cropped_cloud);
					//float noise = calc_noise(x1, x2, cropped_pcl);
					float mid_dist = mid_point_dist(x1, x2);
					std::cout<<"between points "<< start_pt<< " and " << end_pt<< " density is: "<< density <<std::endl;
					//std::cout<<"and dist to origin is: "<<mid_dist<<std::endl;
	//				std::cout<<"noise between points "<< start_pt<< " and " << end_pt<< " is: "<< noise<<std::endl;
					pcl::PointXYZI path_pt;
					
					path_pt.x = t_user_plan_cloud.points[start_pt].x;
					path_pt.y = t_user_plan_cloud.points[start_pt].y;
					path_pt.z = t_user_plan_cloud.points[start_pt].z;
					// float proj_err = calc_projected_err(roll, pitch, mid_dist, density);
					// std::cout<<"projected error is: "<< proj_err<<std::endl;
					if (auto_mode){
						cost_of_current = calc_projected_err(roll, pitch, mid_dist, density);
						std::cout<<"projected error is (including t cost): "<< cost_of_current + auto_to_man_cost << " vs: " << man_cost<<std::endl;
					} else{
						cost_of_next = calc_projected_err(roll, pitch, mid_dist, density);
						std::cout<<"projected error is: "<< cost_of_next<< " vs: " << man_cost + man_to_auto_cost <<std::endl;
					}

					if((cost_of_next < cost_of_current + t_cost) || ((start_mode == "auto") && (pts_id == 0))){
						if ((start_mode == "auto") && (pts_id == 0)){
							auto_mode = true;
						} else if (auto_mode){
							auto_mode = false;
							cost_of_current = man_cost;
							t_cost = man_to_auto_cost;
							//confidence_threshold = man_to_auto_threshold * -1;
						} else{
							auto_mode =true;
							t_cost = auto_to_man_cost;
							cost_of_next = man_cost;
							//confidence_threshold = auto_to_man_threshold;
						}
					}
					if (auto_mode){
						path_pt.intensity = 0;
					} else{
						path_pt.intensity = 1;
					}

					std::cout<<"point intensity is set to: "<< path_pt.intensity<<std::endl;

					if (!edge_only){
						confidence_path_pcl.points.push_back(path_pt);
					} else{
						edge_pcl.points.push_back(path_pt);
						if (pts_id == pts_len - 1){
							if (start_mode == "auto"){
								edge_count ++;
							}

							current_edge_x = edge_pcl.points[edge_count].x;
							current_edge_y = edge_pcl.points[edge_count].y;
							for (int i = 0; i < t_plan_cloud.points.size(); i ++){
								if ((t_plan_cloud.points[i].x == current_edge_x) && (t_plan_cloud.points[i].y == current_edge_y)){
									edge_count ++;
									current_edge_x = edge_pcl.points[edge_count].x;
									current_edge_y = edge_pcl.points[edge_count].y;
								}

								path_pt.x = t_plan_cloud.points[i].x;
								path_pt.y = t_plan_cloud.points[i].y;
								path_pt.z = t_plan_cloud.points[i].z;
								
								if ((edge_pcl.points[edge_count-1].intensity == 0) || ((start_mode == "auto") && (i == 0))){
									path_pt.intensity = 0;
								} else{
									path_pt.intensity = 1;
								}
								confidence_path_pcl.points.push_back(path_pt);
							}
						}
					}


				}

			}

			std_msgs::Header header;
			header.stamp = ros::Time::now();
			//header.seq = seq++;
			header.frame_id = std::string("/camera_color_optical_frame");
			confidence_path_pcl.header = pcl_conversions::toPCL(header);
			if (right_count < 2){
				pub_path_confidence.publish(confidence_path_pcl);
			}

		}
		

		loop_rate.sleep();
		ros::spinOnce();
  }
  return 0;
};