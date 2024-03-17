#pragma once

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/cvdef.h"
#include <pcl/kdtree/kdtree.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>

class myPipeline
{
public:
    myPipeline(ros::NodeHandle &nodeHandle);
    myPipeline();

private:
    void my_callbackPC(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void my_callbackClickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg);
    ros::NodeHandle *m_nodeHandle;
    ros::Subscriber m_PCSubscriber;
    ros::Publisher m_PCMPublisher;
    int pipelineMode;
    pcl::PCLPointCloud2::Ptr cloud;
    pcl::PCLPointCloud2::Ptr cloud_filtered;
    ros::Subscriber m_ClickedPointSubscriber;
    ros::Publisher m_ClusterPublisher;
    bool segmentation;

    volatile float xClicked;
    volatile float yClicked;
    volatile float zClicked;

    bool firstKFRun;
    bool pointClicked;
    cv::KalmanFilter *KF;
};
