//
// Created by yunle on 18-11-26.
//
#include "../include/LidarLaneDetect/LIdarLaneDetecting.h"

#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

//void points_cb(sensor_msgs::PointCloud2::Ptr &origin_cloud);
//void preProcess(pcl::PointCloud<pcl::PointXYZI> &inputCLoud,pcl::PointCloud<pcl::PointXYZI> &outCloud);
//void findCluster(pcl::PointCloud<pcl::PointXYZI> &inputCloud,pcl::PointCloud<pcl::PointXYZI> &outCloud);
//void polyfit(pcl::PointCloud<pcl::PointXYZI> &inputCloud);

int main(int argc,char** argv){
	ros::init(argc,argv,"Lidar_Lane_Detect_node");
	ros::NodeHandle nh;
	ros::NodeHandle privateNh("~");

//	LidarLane::LaneFinding findingLane;
//	findingLane.setup(nh,privateNh);
//	ros::Rate(10);
//	ros::spin();

	return 0;
}

