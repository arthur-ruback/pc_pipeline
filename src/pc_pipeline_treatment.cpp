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
    m_PCSubscriber = m_nodeHandle->subscribe(scanTopicName, scanTopicQueueSize, &myPipeline::my_callbackPC, this);

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

    // get pipeline mode param
    if (!m_nodeHandle->getParam("pipeline_mode", pipelineMode))
    {
        ROS_ERROR("Error getting paramenter pipeline_mode\n");
        return;
    }

    // get segmentation param
    if (!m_nodeHandle->getParam("segmentation", segmentation))
    {
        ROS_ERROR("Error getting paramenter segmentation\n");
        return;
    }

    // get voxelGridSize param
    if (!m_nodeHandle->getParam("voxel_grid_size", voxelGridSize))
    {
        ROS_ERROR("Error getting paramenter voxel_grid_size\n");
        return;
    }

    // get passThroughMin param
    if (!m_nodeHandle->getParam("pass_through_min", passThroughMin))
    {
        ROS_ERROR("Error getting paramenter pass_through_min\n");
        return;
    }

    // get passThroughMax param
    if (!m_nodeHandle->getParam("pass_through_max", passThroughMax))
    {
        ROS_ERROR("Error getting paramenter pass_through_max\n");
        return;
    }

    // get statisticalMean param
    if (!m_nodeHandle->getParam("statistical_mean", statisticalMean))
    {
        ROS_ERROR("Error getting paramenter statistical_mean\n");
        return;
    }

    // get statisticalStdDev param
    if (!m_nodeHandle->getParam("statistical_std_dev", statisticalStdDev))
    {
        ROS_ERROR("Error getting paramenter statistical_std_dev\n");
        return;
    }

    // get clusterTolerance param
    if (!m_nodeHandle->getParam("cluster_tolerance", clusterTolerance))
    {
        ROS_ERROR("Error getting paramenter cluster_tolerance\n");
        return;
    }

    // get minClusterSize param
    if (!m_nodeHandle->getParam("min_cluster_size", minClusterSize))
    {
        ROS_ERROR("Error getting paramenter min_cluster_size\n");
        return;
    }

    // get maxClusterSize param
    if (!m_nodeHandle->getParam("max_cluster_size", maxClusterSize))
    {
        ROS_ERROR("Error getting paramenter max_cluster_size\n");
        return;
    }

    // get kfProcessNoise param
    if (!m_nodeHandle->getParam("kf_process_noise", kfProcessNoise))
    {
        ROS_ERROR("Error getting paramenter kf_process_noise\n");
        return;
    }

    // get kfMeasurementNoise param
    if (!m_nodeHandle->getParam("kf_measure_noise", kfMeasurementNoise))
    {
        ROS_ERROR("Error getting paramenter kf_measurement_noise\n");
        return;
    }

    // subscribe to /clicked_point [geometry_msgs/PointStamped]
    m_ClickedPointSubscriber = m_nodeHandle->subscribe("/clicked_point", 2, &myPipeline::my_callbackClickedPoint, this);

    // create the cluster publisher
    m_ClusterPublisher = m_nodeHandle->advertise<sensor_msgs::PointCloud2>("/cluster", 1);

    // Create a PCLPointCloud2
    cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_filtered = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);

    xClicked = yClicked = zClicked = 0;
    firstKFRun = true;
    pointClicked = false;

    KF = NULL;

    ROS_INFO("Successfully launched node.");
}

// main pipeline callback function
void myPipeline::my_callbackPC(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // new conversion to use with XYZ cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *in_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filteredXYZ(new pcl::PointCloud<pcl::PointXYZ>);

    // Perform the actual filtering based on pipeline mode
    switch (pipelineMode)
    {
    case 0:
    {
        // Voxel Grid filter
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
        sor.filter(*cloud_filtered);
        // Convert to XYZ cloud
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filteredXYZ);
        break;
    }
    case 1:
    {
        // Pass Through filter
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passThroughMin, passThroughMax);
        pass.filter(*cloud_filtered);
        // Convert to XYZ cloud
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filteredXYZ);
        break;
    }
    case 2:
    {
        // Pass Through and Voxel Grid filters
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passThroughMin, passThroughMax);
        pass.filter(*cloud_filtered);
        pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
        vox.setInputCloud(cloud_filtered);
        vox.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
        vox.filter(*cloud_filtered);
        // Convert to XYZ cloud
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filteredXYZ);
        break;
    }
    case 3:
    {
        // Pass Through and Voxel Grid filters
        pcl::PassThrough<pcl::PCLPointCloud2> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(passThroughMin, passThroughMax);
        pass.filter(*cloud_filtered);
        pcl::VoxelGrid<pcl::PCLPointCloud2> vox;
        vox.setInputCloud(cloud_filtered);
        vox.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize);
        vox.filter(*cloud_filtered);
        // Convert to XYZ cloud
        pcl::fromPCLPointCloud2(*cloud_filtered, *cloud_filteredXYZ);
        // Statistical Outlier Removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_filteredXYZ);
        sor.setMeanK(statisticalMean);
        sor.setStddevMulThresh(statisticalStdDev);
        sor.filter(*cloud_filteredXYZ);
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
    // pcl_conversions::moveFromPCL(*cloud_filtered, output);
    pcl::toROSMsg(*cloud_filteredXYZ.get(), output);

    // Publish the data
    m_PCMPublisher.publish(output);

    // check if clicked point is valid to either initialize the kalman filter or update it
    if (segmentation && pointClicked)
    {
        if (firstKFRun)
        {
            if (KF != NULL)
            {
                delete KF;
            }

            // create Kalman filter cv2
            KF = new cv::KalmanFilter(4, 2);

            // initialize kalman filter
            ROS_INFO("(Re)Initializing Kalman Filter");
            firstKFRun = false;

            // Kalman states
            // [x, y, v_x, v_y]
            KF->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                    0, 1, 0, 1,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1);
            float sigmaP = kfProcessNoise;     // process noise covariance
            float sigmaQ = kfMeasurementNoise; // measurement noise covariance
            cv::setIdentity(KF->measurementMatrix);
            cv::setIdentity(KF->processNoiseCov, cv::Scalar::all(sigmaP));
            cv::setIdentity(KF->measurementNoiseCov, cv::Scalar(sigmaQ));
            cv::setIdentity(KF->errorCovPost, cv::Scalar::all(1));

            // Set initial state
            // FIXME: LOOK INTO FRAME TRANSFORMATIONS, SPECIALLY WHEN THE ROBOT WILL MOVE
            KF->statePre.at<float>(0) = yClicked;
            KF->statePre.at<float>(1) = xClicked;
            KF->statePre.at<float>(2) = 0; // initial v_x
            KF->statePre.at<float>(3) = 0; // initial v_y
            KF->statePost.at<float>(0) = yClicked;
            KF->statePost.at<float>(1) = xClicked;
            KF->statePost.at<float>(2) = 0; // initial v_x
            KF->statePost.at<float>(3) = 0; // initial v_y

            // fisrt KF run
            cv::Mat prediction = KF->predict();
            cv::Mat measurement = (cv::Mat_<float>(2, 1) << yClicked, xClicked);
            KF->correct(measurement);
            ROS_INFO("KF Estimation: (%f,%f)", prediction.at<float>(0), prediction.at<float>(1));
            ROS_INFO("Measurement  : (%f,%f)", measurement.at<float>(1), measurement.at<float>(0));
        }
        else
        {
            // SEGMENT THE POINT CLOUD
            /* Creating the KdTree from input point cloud*/
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filteredXYZ);

            /* Here we are creating a vector of PointIndices, which contains the actual
             * index information in a vector<int>. The indices of each detected cluster
             * are saved here. Cluster_indices is a vector containing one instance of
             * PointIndices for each detected cluster. Cluster_indices[0] contain all
             * indices of the first cluster in input point cloud.
             */
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(clusterTolerance);
            ec.setMinClusterSize(minClusterSize);
            ec.setMaxClusterSize(maxClusterSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filteredXYZ);
            /* Extract the clusters out of pc and save indices in cluster_indices.*/
            ec.extract(cluster_indices);

            /* To separate each cluster out of the vector<PointIndices> we have to
             * iterate through cluster_indices, create a new PointCloud for each
             * entry and write all points of the current cluster in the PointCloud.
             */

            std::vector<pcl::PointIndices>::const_iterator it;
            std::vector<int>::const_iterator pit;
            // Vector of cluster pointclouds
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

            // Cluster centroids
            std::vector<pcl::PointXY> clusterCentroids;

            for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
            {
                float x = 0.0;
                float y = 0.0;
                int numPts = 0;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                    new pcl::PointCloud<pcl::PointXYZ>);
                for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
                {

                    cloud_cluster->points.push_back(cloud_filteredXYZ->points[*pit]);

                    x += cloud_filteredXYZ->points[*pit].x;
                    y += cloud_filteredXYZ->points[*pit].y;
                    numPts++;
                }

                pcl::PointXY centroid;
                centroid.x = x / numPts;
                centroid.y = y / numPts;

                cluster_vec.push_back(cloud_cluster);

                // Get the centroid of the cluster
                clusterCentroids.push_back(centroid);
            }

            // FIND THE CLOSEST CLUSTER TO THE KF ESTIMATION (OR LAST ESTIMATION)
            // TODO: MAYBE CORRELATE THE NUMBER OF POITNS TO FILTER THE CLOUDS
            float minDist = FLT_MAX;
            int rightCluserIndice = -1;
            cv::Mat prediction = KF->predict();
            // FIXME: LOOK INTO FRAME TRANSFORMATIONS, SPECIALLY WHEN THE ROBOT WILL MOVE
            float kfEst_x = prediction.at<float>(1);
            float kfEst_y = prediction.at<float>(0);
            // float kfEst_x = yClicked; // REMOVE: PROVIsoire
            // float kfEst_y = xClicked; // REMOVE: PROVIsoire
            for (unsigned i = 0; i < clusterCentroids.size(); i++)
            {
                float cluster_x = clusterCentroids[i].x;
                float cluster_y = clusterCentroids[i].y;
                float dist = sqrt(pow(cluster_x - kfEst_x, 2) + pow(cluster_y - kfEst_y, 2));
                if (dist < minDist)
                {
                    minDist = dist;
                    rightCluserIndice = i;
                }
            }

            if (rightCluserIndice == -1)
            {
                ROS_ERROR("No cluster found");
                return;
            }

            ROS_INFO("Cluster with %ld points", cluster_vec[rightCluserIndice]->points.size());

            // GET NEW CENTROIDE FROM THE CLUSTER
            // FIXME: LOOK INTO FRAME TRANSFORMATIONS, SPECIALLY WHEN THE ROBOT WILL MOVE
            cv::Mat measurement = (cv::Mat_<float>(2, 1) << clusterCentroids[rightCluserIndice].y, clusterCentroids[rightCluserIndice].x);

            // UPDATE THE KF WITH NEW DATA
            KF->correct(measurement);
            ROS_INFO("KF Estimation: (%f,%f)", kfEst_x, kfEst_y);
            ROS_INFO("Measurement  : (%f,%f)", measurement.at<float>(1), measurement.at<float>(0));

            // PUBLISH POINTS FROM OBJECT CLUSTER
            pcl::PCLPointCloud2::Ptr cloud_cluster(new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2(*cluster_vec[rightCluserIndice], *cloud_cluster);

            // Convert to ROS data type
            sensor_msgs::PointCloud2 output_cluster_msg;
            pcl_conversions::moveFromPCL(*cloud_cluster, output_cluster_msg);
            output_cluster_msg.header.frame_id = msg->header.frame_id;
            output_cluster_msg.header.stamp = ros::Time::now();

            // Publish the data
            m_ClusterPublisher.publish(output_cluster_msg);
        }
    }
}

// callback funtion to clicked point
void myPipeline::my_callbackClickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    ROS_INFO("Clicked point received");
    // print clicked point data
    ROS_INFO("x: %f, y: %f, z: %f", msg->point.x, msg->point.y, msg->point.z);

    // store clicked point data
    xClicked = msg->point.x;
    yClicked = msg->point.y;
    zClicked = msg->point.z;

    firstKFRun = true;
    pointClicked = true;
}
