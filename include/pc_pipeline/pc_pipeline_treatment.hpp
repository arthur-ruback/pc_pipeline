#pragma once

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class myPipeline
{
public:
    myPipeline(ros::NodeHandle &nodeHandle);
    myPipeline();

private:
    void my_callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    ros::NodeHandle *m_nodeHandle;
    ros::Subscriber m_PCSubscriber;
    ros::Publisher m_PCMPublisher;
    int pipelineMode;
    pcl::PCLPointCloud2::Ptr cloud;
    pcl::PCLPointCloud2::Ptr cloud_filtered;
};