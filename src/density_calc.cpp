#include <ros/ros.h>
#include <tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <surgery_sim/Plan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <surgery_sim/Density.h>
#include <list>

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
pcl::PointCloud<pcl::PointXYZ> selected_cloud;
pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
bool pcl_received = false;
bool cropped_received = false;
bool selected_received = false;

sensor_msgs::PointCloud2 test;
list<float> edge1;
list<float> edge2;
list<float> edge3;
list<float> edge4;


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
	cout <<"read" << pcd.points.size()<< " points from the pcd"<<endl;
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
			 cout <<"start point (cap) " <<  x1<<endl;
			 cout <<"end point (cap) " <<  x2<<endl;

				cout <<"test point distance from the line " << distance<< " meters ---- height is: " <<h<<" meters"<<endl;

		}
 		ctr ++;
	}


 	 double cyl_volume = M_PI*r_c*r_c*h*1000000000;//volume of the cylinder formed by x1-x2 line and radius r_c
 // cout << "volume was: " << cyl_volume<< " for h= "<< h << endl; 
	//return density/pcd.points.size();
	return density/cyl_volume;
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
  ros::init(argc, argv, "density_calc");
  ros::NodeHandle node;

	ros::Subscriber plan_cloud_sub = node.subscribe("/plancloud", 1, pcl_callback);
	ros::Subscriber crop_cloud_sub = node.subscribe("/cropped_pointcloud", 1, cropped_pcl_callback);
	ros::Subscriber selected_cloud_sub = node.subscribe("/selected_points", 1, selected_pcl_callback);
	//ros::Subscriber crop_cloud_sub = node.subscribe("/camera/depth_registered/points", 1, cropped_pcl_callback);

	ros::Publisher pub_density= node.advertise<surgery_sim::Density>( "/density", 1);

  tf::TransformListener listener;
	geometry_msgs::Vector3 old_v;
	geometry_msgs::Vector3 new_v;
	float den;
	float den_avg;
	int ctr = 0;
	bool flag = true;
	std_msgs::Float32 den_avg32;
  

  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  while (node.ok()){
		if (selected_received && cropped_received){
			vector<std_msgs::Float32> densities;
			//if (flag){
				for(int i = 0; i < selected_cloud.points.size(); i++){
					if (i == 0){
						old_v.x = selected_cloud.points[i].x;
						old_v.y = selected_cloud.points[i].y;
						old_v.z = selected_cloud.points[i].z;
					} else{
						new_v.x = selected_cloud.points[i].x;
						new_v.y = selected_cloud.points[i].y;
						new_v.z = selected_cloud.points[i].z;

						den = (calc_density(old_v, new_v, cropped_cloud));

						//cout << "density from point " << i-1 << " and " << i << ": " << den << endl;

						// if (ctr < 5{
						// 	den_avg += den;
						// 	ctr++;
						// }) else {
						// 	density / 5;
						// 	cout << "average density from point " << i-1 << " and " << i << ": " << den << endl;
						// 	ctr = 0;
						// 	den_avg = 0;
						// }

						cout << "old point " << i-1 << ": " << old_v << " new point " << i << ": " << new_v << endl;

						switch(i){
							case 1:
								edge1.push_front(den);
								if (edge1.size() > 5){
									edge1.pop_back();
								}
								for (it = edge1.begin(); it != edge1.end(); ++it){
									den_avg += *it;
								}
								//den_avg32.data = den_avg / edge1.size();
								den_avg32.data = den;
								density.values.push_back(den_avg32);
								
								break;
							case 2:
								edge2.push_front(den);
								if (edge2.size() > 5){
									edge2.pop_back();
								}
								for (it = edge2.begin(); it != edge2.end(); ++it){
									den_avg += *it;
								}
								//den_avg32.data = den_avg / edge1.size();
								den_avg32.data = den;
								density.values.push_back(den_avg32);
								
								break;
							case 3:
								edge3.push_front(den);
								if (edge3.size() > 5){
									edge3.pop_back();
								}
								for (it = edge3.begin(); it != edge3.end(); ++it){
									den_avg += *it;
								}
								//den_avg32.data = den_avg / edge1.size();
								den_avg32.data = den;
								density.values.push_back(den_avg32);
								
								break;
							case 4:
								edge4.push_front(den);
								if (edge4.size() > 5){
									edge4.pop_back();
								}
								for (it = edge4.begin(); it != edge4.end(); ++it){
									den_avg += *it;
								}
								//den_avg32.data = den_avg / edge1.size();
								den_avg32.data = den;
								density.values.push_back(den_avg32);
								
								break;
						}

						old_v.x = new_v.x;
						old_v.y = new_v.y;
						old_v.z = new_v.z;
						den_avg = 0;
					}
					flag = false;
				}
				//density.values = densities;
				pub_density.publish(density);
				density.values.clear();
			//}
		}

	loop_rate.sleep();
	ros::spinOnce();
  }
  return 0;
};