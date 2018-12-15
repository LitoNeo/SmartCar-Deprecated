#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"load_global_map_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    sensor_msgs::PointCloud2 msg_map;
    ros::Duration duration;

    std::string param_pcd_file_path;
    std::string param_map_frame;
    double param_duration;  // 用以定义发布地图打频率，等同ros::rate();

    pnh.param<std::string>("pcd_file_path",param_pcd_file_path,"");
    pnh.param<std::string>("map_frame",param_map_frame,"/map");
    pnh.param<double>("duration",param_duration,1.0);

    if (param_pcd_file_path == "" || pcl::io::loadPCDFile(param_pcd_file_path,msg_map) == -1){
        ROS_ERROR("Failed to load map pcd file: %s",param_pcd_file_path);
        return (-1);
    }
    msg_map.header.frame_id = param_map_frame;

    ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/static_map",1);
    duration.fromSec(param_duration);

    while(ros::ok()){
        msg_map.header.stamp = ros::Time::now();
        pub_map.publish(msg_map);
        
        duration.sleep();
    }

    return 0;
}