//
// Created by yunle on 18-11-26.
//

#ifndef LIDARLANEDETECT_LIDARLANEDETECTING_H
#define LIDARLANEDETECT_LIDARLANEDETECTING_H

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>

namespace LidarLane{
	typedef pcl::PointXYZ  PointT;
	typedef pcl::PointXYZI PointI;

	struct pose{
		double LRaway;
	};

	class LaneFinding{
		private:
			typedef boost::shared_ptr<LaneFinding> Ptr;

			pcl::PointCloud<PointI>::Ptr inputCloud_ptr;

	  	double laneLeftBoundary;
			double laneRightBoundary;

			double minClipHeight;
			double maxClipHeight;

			bool _removeCloseNoise = false;
			double _closeNoiseDistance = 0.3;

			bool _downsample = true;
			double _leafSize = 0.5;

			bool _clipHeigt = true;
			bool _clipBoundary = true;

			double _maxFloorAngle = 1;
			double _maxFloorHeight = 0.5;


		private:
			pcl::PointCloud<pcl::PointXYZ>::Ptr te;

			ros::Subscriber cloud_sub;
			ros::Publisher roadCloud_pub;
			ros::Publisher pose_pub;
			ros::Publisher laneCloud_pub;

		public:
			LaneFinding():inputCloud_ptr(new pcl::PointCloud<pcl::PointXYZI>){};

			void setup(ros::NodeHandle &handle,ros::NodeHandle &privateHandle);

			void points_cb(sensor_msgs::PointCloud2::ConstPtr input_cloud);

		  void downsampleCloud(pcl::PointCloud<PointI>::Ptr inputCloud, pcl::PointCloud<PointI>::Ptr outCloud);

		  void polyfit(pcl::PointCloud<PointI> inputCloud);

			void removeCloseRearNoise(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void clipHeight(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void clipBoundary(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void extractRoad(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<PointI>::Ptr outCloud);

			void extractLane(pcl::PointCloud<PointI>::Ptr inputCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud);

	};
}

#endif //LIDARLANEDETECT_LIDARLANEDETECTING_H
