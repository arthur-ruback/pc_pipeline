#include "pc_pipeline/pc_pipeline_treatment.hpp"

myPipeline::myPipeline(ros::NodeHandle &nodeHandle) : m_nodeHandle(&nodeHandle)
{
    std::string scanTopicName;
    int scanTopicQueueSize;
    if (!m_nodeHandle->getParam("scan_name", scanTopicName))
    {
        ROS_ERROR("Error getting paramenter scan/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("scan_queue_size", scanTopicQueueSize))
    {
        ROS_ERROR("Error getting paramenter scan/queue_size\n");
        return;
    }
    m_PCSubscriber = m_nodeHandle->subscribe(scanTopicName, scanTopicQueueSize, &myPipeline::my_callback, this);

    std::string scanModifiedTopicName;
    int scanModifiedTopicQueueSize;
    if (!m_nodeHandle->getParam("scan_modified_name", scanModifiedTopicName))
    {
        ROS_ERROR("Error getting paramenter scan_modified/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("scan_modified_queue_size", scanModifiedTopicQueueSize))
    {
        ROS_ERROR("Error getting paramenter scan_modified/queue_size\n");
        return;
    }
    m_PCMPublisher = m_nodeHandle->advertise<sensor_msgs::PointCloud2>(scanModifiedTopicName, scanModifiedTopicQueueSize);

    // get pipeline mode
    if (!m_nodeHandle->getParam("pipeline_mode", pipelineMode))
    {
        ROS_ERROR("Error getting paramenter pipeline_mode\n");
        return;
    }

    // Create a PCLPointCloud2
    cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_filtered = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);

    ROS_INFO("Successfully launched node.");
}

// callback function that copies the input point cloud and publishes it
void myPipeline::my_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // sensor_msgs::PointCloud2 msgCopy;
    // msgCopy.header = msg->header;
    // msgCopy.height = msg->height;
    // msgCopy.width = msg->width;
    // msgCopy.fields = msg->fields;
    // msgCopy.is_bigendian = msg->is_bigendian;
    // msgCopy.point_step = msg->point_step;
    // msgCopy.row_step = msg->row_step;
    // msgCopy.is_dense = msg->is_dense;
    // msgCopy.data = msg->data;

    // m_PCMPublisher.publish(msgCopy);

    // Container for original & filtered data

    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // Perform the actual filtering based on pipeline mode
    switch (pipelineMode)
    {
    case 0:
    {
        // Voxel Grid filter
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.1, 0.1, 0.1);
        sor.filter(*cloud_filtered);
        break;
    }
    case 1:
    {
        // Pass Through filter
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.4, 1.3);
        pass.filter(*cloud_filtered);
        break;
    }
    case 2:
    {
        // Pass Through and Voxel Grid filters
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.4, 1.3);
        pass.filter(*cloud_filtered);

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud_filtered);
        sor.setLeafSize(0.1, 0.1, 0.1);
        sor.filter(*cloud_filtered);
        break;
    }
    default:
    {
        ROS_ERROR("Invalid pipeline mode\n");
        return;
    }
    }

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::moveFromPCL(*cloud_filtered, output);

    // Publish the data
    m_PCMPublisher.publish(output);
}
