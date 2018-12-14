//
// Created by yunle on 18-11-26.
//

#include "../include/LidarLaneDetect/LIdarLaneDetecting.h"


namespace LidarLane{

	void LaneFinding::setup(ros::NodeHandle &handle, ros::NodeHandle &privateHandle) {
		handle.param<double>("laneLeftBoundary",laneLeftBoundary,5.0);
		handle.param<double>("laneRightBoundary",laneRightBoundary,5.0);
		handle.param<double>("minClipHeight",minClipHeight,-5.0);
		handle.param<double>("maxClipHeight",maxClipHeight,0.3);

		ROS_INFO("\nlaneLeftBoundary:%.2lf \n"
						 "laneRightBoundary:%.2f\n"
						 "minClipHeight:%.2f\n"
						 "maxClipHeight:%.2f",laneLeftBoundary,laneRightBoundary,minClipHeight,maxClipHeight);

		roadCloud_pub = handle.advertise<sensor_msgs::PointCloud2>("LidarRoad_cloud",100);
		laneCloud_pub = handle.advertise<sensor_msgs::PointCloud2>("LidarLane_cloud",100);
//		pose_pub = handle.advertise<>("pose_byLidar",1);

		cloud_sub = handle.subscribe("/velodyne_points",10000,&LaneFinding::points_cb,this);
	}

	void LaneFinding::points_cb(sensor_msgs::PointCloud2::ConstPtr input_cloud) {
		pcl::fromROSMsg(*input_cloud,*inputCloud_ptr);
		pcl::PointCloud<PointI>::Ptr downsampledCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr noiseRemovedCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr clipedHeightCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr clipedBoundaryCloud(new pcl::PointCloud<PointI>);
		pcl::PointCloud<PointI>::Ptr onlyRoadCloud(new pcl::PointCloud<PointI>);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr laneCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

		if(_downsample){
			downsampleCloud(inputCloud_ptr,downsampledCloud);
		}else{
			downsampledCloud = inputCloud_ptr;
		}

		if(_removeCloseNoise){
			removeCloseRearNoise(downsampledCloud,noiseRemovedCloud);
		}else{
			noiseRemovedCloud = downsampledCloud;
		}

		if(_clipHeigt){
			clipHeight(noiseRemovedCloud,clipedHeightCloud);
		}else{
			clipedHeightCloud = noiseRemovedCloud;
		}

		if(_clipBoundary){
			clipBoundary(clipedHeightCloud,clipedBoundaryCloud);
		}else{
			clipedBoundaryCloud = clipedHeightCloud;
		}

		// 提取路面并发布
		extractRoad(clipedBoundaryCloud,onlyRoadCloud);
		sensor_msgs::PointCloud2 RoadCloud2;
		pcl::toROSMsg(*onlyRoadCloud,RoadCloud2);
		RoadCloud2.header.frame_id="odom";
		roadCloud_pub.publish(RoadCloud2);

		// 提取车道线并发布 --lane为带RGB信息的PointCloud2
		extractLane(onlyRoadCloud,laneCloud);
		sensor_msgs::PointCloud2 laneRGB;
		pcl::toROSMsg(*laneCloud,laneRGB);
		laneRGB.header.frame_id = "odom";
		laneCloud_pub.publish(laneRGB);
		ROS_INFO("laneRGB size = %ld",laneRGB.data.size());
	}

	void LaneFinding::polyfit(pcl::PointCloud<PointI> inputCloud) {

	}

	void LaneFinding::removeCloseRearNoise(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			double distance = std::pow(point.x,2) + std::pow(point.y,2);
			if(distance >= _closeNoiseDistance*_closeNoiseDistance)
				outCloud->points.push_back(point);
		}

	}

	void LaneFinding::downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		pcl::VoxelGrid<PointI> voxelGrid;
		voxelGrid.setLeafSize(_leafSize,_leafSize,_leafSize);
		voxelGrid.setInputCloud(inputCloud);
		voxelGrid.filter(*outCloud);
	}

	void LaneFinding::clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			if(minClipHeight <= point.z && point.z <= maxClipHeight){
				outCloud->points.push_back(point);
			}
		}
	}

	void LaneFinding::clipBoundary(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		outCloud->points.clear();
		for(auto point:inputCloud->points){
			if(-laneRightBoundary <= point.x && point.x <= laneLeftBoundary){  // velodyne采集的数据点为<右手坐标系,但y轴和数据线平齐>(影响车身朝向)
				outCloud->points.push_back(point);
			}
		}
	}

	void LaneFinding::extractRoad(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud) {
		pcl::SACSegmentation<PointI> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(100);
		seg.setAxis(Eigen::Vector3f(0,0,1));
		seg.setEpsAngle(_maxFloorAngle);
//		seg.setDistanceThreshold(_maxFloorHeight);  // 判断是否为模型内点的距离阈值
		seg.setDistanceThreshold(0.15);

		seg.setInputCloud(inputCloud);
		seg.segment(*inliers,*coefficients);
		if(inliers->indices.empty()){
			std::cout<<"Could not extract the road(plane)"<<std::endl;
		}

		// pcl/filters/extract_indices.h
		pcl::ExtractIndices<PointI> extract;
		extract.setInputCloud(inputCloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*outCloud);
	}

	void
	LaneFinding::extractLane(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud) {
		outCloud->points.clear();
//		for(auto point:inputCloud->points){
//			if(point.intensity >= 30){
//				pcl::PointXYZRGB tmp;
//				tmp.x = point.x;
//				tmp.y = point.y;
//				tmp.z = point.z;
//				tmp.r = 255;
//				tmp.g = 0;
//				tmp.b = 0;
//				outCloud->points.push_back(tmp);
//			}
//		}
//	}

//		pcl::PointCloud<PointI>::Ptr copyCloud;
//		pcl::copyPointCloud(*inputCloud,*copyCloud);

//		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		// 以下采用整体intensity均值,效果有待验证
		double sum_intensity = 0.0;
		for(auto point:inputCloud->points){
			sum_intensity += point.intensity;
		}
		double thresh = 0.0;
		if(!inputCloud->points.empty()){
			thresh = sum_intensity / inputCloud->points.size();
			ROS_WARN("thresh = %.2f",thresh);
		}else{
			ROS_WARN("Detect No point in roadCloud");
		}

		for(auto point:inputCloud->points){
			if(point.intensity > thresh){
				pcl::PointXYZRGB tmp;
				tmp.x = point.x;
				tmp.y = point.y;
				tmp.z = point.z;
				tmp.r = 255;
				tmp.g = 0;
				tmp.b = 0;
				outCloud->points.push_back(tmp);
			}
		}
	}

}