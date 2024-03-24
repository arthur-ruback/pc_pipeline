#pragma once

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
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

#include <mutex>

#define DT 0.1

class myPipeline
{
public:
    myPipeline(ros::NodeHandle &nodeHandle);
    myPipeline();

private:
    // Callbacks
    void cb_cloudFilter(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void cb_clickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg);
    void cb_kalman_update(const ros::TimerEvent &event);
    void cb_poseMeasurement(const geometry_msgs::PoseArray::ConstPtr &msg);

    // ROS communication
    ros::NodeHandle *m_nodeHandle;
    ros::Subscriber m_PCSubscriber;
    ros::Subscriber m_ClickedPointSubscriber;
    ros::Subscriber m_poseMeasurementSubscriber;
    ros::Publisher m_PCFilteredPublisher;
    ros::Publisher m_ClusterCenterPublisher;
    ros::Publisher m_PCCandiadtesPublisher;

    // used for treatment
    pcl::PCLPointCloud2::Ptr cloud;
    pcl::PCLPointCloud2::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_xyz;

    // Kalman filter
    cv::KalmanFilter *KF;
    std::mutex *mtx;
    std::string KFframeID;

    // params from launch file
    int pipelineMode;
    bool segmentation;
    float voxelGridSize;
    float passThroughMin;
    float passThroughMax;
    float statisticalMean;
    float statisticalStdDev;
    float clusterTolerance;
    int minClusterSize;
    int maxClusterSize;
    float kfProcessNoise;
    float kfMeasurementNoise;
    int measurementFailThreshold;

    // Other
    ros::Timer m_timer;
    int printCounter;
    int measurementFailCounter;
};
