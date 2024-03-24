#include "pc_pipeline/pc_pipeline_treatment.hpp"

myPipeline::myPipeline(ros::NodeHandle &nodeHandle) : m_nodeHandle(&nodeHandle)
{
    // subscriber for the point cloud
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
    m_PCSubscriber = m_nodeHandle->subscribe(scanTopicName, scanTopicQueueSize, &myPipeline::cb_cloudFilter, this);

    // publisher for the filtered point cloud
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
    m_PCFilteredPublisher = m_nodeHandle->advertise<sensor_msgs::PointCloud2>(scanModifiedTopicName, scanModifiedTopicQueueSize);

    // publisher for the candidates point cloud
    std::string scanCandidatesTopicName;
    int scanCandidatesTopicQueueSize;
    if (!m_nodeHandle->getParam("scan_candidates_name", scanCandidatesTopicName))
    {
        ROS_ERROR("Error getting paramenter scan_candidates/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("scan_candidates_queue_size", scanCandidatesTopicQueueSize))
    {
        ROS_ERROR("Error getting paramenter scan_candidates/queue_size\n");
        return;
    }
    m_PCCandiadtesPublisher = m_nodeHandle->advertise<sensor_msgs::PointCloud2>(scanCandidatesTopicName, scanCandidatesTopicQueueSize);

    // subscriber for the pose measurement
    std::string poseMeasurementTopicName;
    int poseMeasurementTopicQueueSize;
    if (!m_nodeHandle->getParam("pose_measurement_name", poseMeasurementTopicName))
    {
        ROS_ERROR("Error getting paramenter pose_measurement/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("pose_measurement_queue_size", poseMeasurementTopicQueueSize))
    {
        ROS_ERROR("Error getting paramenter pose_measurement/queue_size\n");
        return;
    }
    m_poseMeasurementSubscriber = m_nodeHandle->subscribe(poseMeasurementTopicName, poseMeasurementTopicQueueSize, &myPipeline::cb_poseMeasurement, this);

    // subscribe to clicked point
    std::string clickedPointTopicName;
    int clickedPointTopicQueueSize;
    if (!m_nodeHandle->getParam("clicked_point_name", clickedPointTopicName))
    {
        ROS_ERROR("Error getting paramenter clicked_point/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("clicked_point_queue_size", clickedPointTopicQueueSize))
    {
        ROS_ERROR("Error getting paramenter clicked_point/queue_size\n");
        return;
    }
    m_ClickedPointSubscriber = m_nodeHandle->subscribe(clickedPointTopicName, clickedPointTopicQueueSize, &myPipeline::cb_clickedPoint, this);

    // create the cluster center publisher
    int clusterCenterQueueSize;
    std::string clusterCenterTopicName;
    if (!m_nodeHandle->getParam("cluster_center_name", clusterCenterTopicName))
    {
        ROS_ERROR("Error getting paramenter cluster_center/name\n");
        return;
    }
    if (!m_nodeHandle->getParam("cluster_center_queue_size", clusterCenterQueueSize))
    {
        ROS_ERROR("Error getting paramenter cluster_center/queue_size\n");
        return;
    }
    m_ClusterCenterPublisher = m_nodeHandle->advertise<geometry_msgs::PointStamped>(clusterCenterTopicName, clusterCenterQueueSize);

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

    // get measurementFailThreshold param
    if (!m_nodeHandle->getParam("measurement_fail_threshold", measurementFailThreshold))
    {
        ROS_ERROR("Error getting paramenter measurement_fail_threshold\n");
        return;
    }

    // initialize the counters
    printCounter = 0;
    measurementFailCounter = 0;

    // Create the point cloud data storage
    cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_filtered = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
    cloud_filtered_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    KF = NULL;

    // create a timer to update the kalman filter
    m_timer = m_nodeHandle->createTimer(ros::Duration(DT), &myPipeline::cb_kalman_update, this);

    // initialize mutex
    mtx = new std::mutex();

    ROS_INFO("Successfully launched node.");
}

// filter callback function
void myPipeline::cb_cloudFilter(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    KFframeID = msg->header.frame_id;
    // Convert to PCL data type
    pcl_conversions::toPCL(*msg, *cloud);

    // new conversion to use with XYZ cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filteredXYZ(new pcl::PointCloud<pcl::PointXYZI>);

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
        // Pass Through and Voxel Grid and statistical outliner filters
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
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
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
    m_PCFilteredPublisher.publish(output);

    // check if clicked point is valid to either initialize the kalman filter or update it
    if (segmentation)
    {
        // SEGMENT THE POINT CLOUD
        /* Creating the KdTree from input point cloud*/
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud_filteredXYZ);

        /* Here we are creating a vector of PointIndices, which contains the actual
         * index information in a vector<int>. The indices of each detected cluster
         * are saved here. Cluster_indices is a vector containing one instance of
         * PointIndices for each detected cluster. Cluster_indices[0] contain all
         * indices of the first cluster in input point cloud.
         */
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
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
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_vec;

        // Cluster centroids
        std::vector<pcl::PointXY> clusterCentroids;

        for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            float x = 0.0;
            float y = 0.0;
            int numPts = 0;
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(
                new pcl::PointCloud<pcl::PointXYZI>);
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

        // combine all the candidates into a single point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZI>);
        for (unsigned i = 0; i < cluster_vec.size(); i++)
        {
            for (unsigned j = 0; j < cluster_vec[i]->points.size(); j++)
            {
                candidates->points.push_back(cluster_vec[i]->points[j]);
            }
        }

        // Convert to ROS data type
        sensor_msgs::PointCloud2 output_candidates;
        pcl::toROSMsg(*candidates.get(), output_candidates);

        // Set the frame ID
        output_candidates.header.frame_id = msg->header.frame_id;
        output_candidates.header.stamp = ros::Time::now();

        // Publish the data
        m_PCCandiadtesPublisher.publish(output_candidates);
    }
}

// callback funtion to clicked point
void myPipeline::cb_clickedPoint(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    ROS_INFO("Clicked point received");
    // print clicked point data
    ROS_INFO("x: %f, y: %f, z: %f", msg->point.x, msg->point.y, msg->point.z);

    mtx->lock();
    if (KF != NULL)
    {
        delete KF;
    }

    // create Kalman filter cv2
    KF = new cv::KalmanFilter(4, 2);

    // initialize kalman filter
    ROS_INFO("(Re)Initializing Kalman Filter");

    // Kalman states
    // [x, y, v_x, v_y]
    KF->transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, DT, 0,
                            0, 1, 0, DT,
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
    KF->statePre.at<float>(0) = msg->point.x;
    KF->statePre.at<float>(1) = msg->point.y;
    KF->statePre.at<float>(2) = 0; // initial v_x
    KF->statePre.at<float>(3) = 0; // initial v_y
    KF->statePost.at<float>(0) = msg->point.x;
    KF->statePost.at<float>(1) = msg->point.y;
    KF->statePost.at<float>(2) = 0; // initial v_x
    KF->statePost.at<float>(3) = 0; // initial v_y

    mtx->unlock();
}

// callback function to update the kalman filter
void myPipeline::cb_kalman_update(const ros::TimerEvent &)
{
    mtx->lock();
    if (KF != NULL)
    {
        printCounter++;
        cv::Mat prediction = KF->predict();
        // PUBLISH THE CLUSTER CENTROID
        geometry_msgs::PointStamped clusterCenter;
        clusterCenter.header.frame_id = KFframeID;
        clusterCenter.header.stamp = ros::Time::now();
        clusterCenter.point.x = KF->statePost.at<float>(0);
        clusterCenter.point.y = KF->statePost.at<float>(1);
        clusterCenter.point.z = 0;
        m_ClusterCenterPublisher.publish(clusterCenter);
        if (printCounter >= 10)
        {
            printCounter = 0;
            /*ROS_INFO("KF Estimation: (%f,%f), KF Covariance: (%f,%f)",
                     KF->statePost.at<float>(1),
                     KF->statePost.at<float>(0),
                     KF->errorCovPost.at<float>(0, 0),
                     KF->errorCovPost.at<float>(1, 1));
            */
        }
    }
    mtx->unlock();
}

// callback function to get a measument to the kalman filter
void myPipeline::cb_poseMeasurement(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    // ROS_INFO("Measurement received");

    if (measurementFailCounter >= measurementFailThreshold)
    {
        ROS_ERROR("Too many failed measurements, resetting KF");
        measurementFailCounter = 0;
        mtx->lock();
        if (KF != NULL)
        {
            delete KF;
            KF = NULL;
        }
        mtx->unlock();
        return;
    }

    // check if the kalman filter is initialized
    if (KF != NULL)
    {
        // Go through all the poses and get the closest one within a certain threshold
        float minDist = FLT_MAX;
        int rightCluserIndice = -1;
        // FIXME: LOOK INTO FRAME TRANSFORMATIONS, SPECIALLY WHEN THE ROBOT WILL MOVE
        float kfEst_x = KF->statePost.at<float>(0);
        float kfEst_y = KF->statePost.at<float>(1);
        float requiredDist = 2 * sqrt(KF->errorCovPost.at<float>(0, 0) + KF->errorCovPost.at<float>(1, 1));
        for (unsigned i = 0; i < msg->poses.size(); i++)
        {
            float cluster_x = msg->poses[i].position.x;
            float cluster_y = msg->poses[i].position.y;
            float dist = sqrt(pow(cluster_x - kfEst_x, 2) + pow(cluster_y - kfEst_y, 2));
            if (dist < minDist && dist < requiredDist)
            {
                minDist = dist;
                rightCluserIndice = i;
            }
        }

        // if minDist is too large, then no cluster was found
        if (rightCluserIndice == -1)
        {
            ROS_WARN("Discarding measurement, no CLOSE cluster found");
            measurementFailCounter++;
            return;
        }

        measurementFailCounter = 0;

        // UPDATE THE KF WITH NEW DATA
        cv::Mat measurement = (cv::Mat_<float>(2, 1) << msg->poses[rightCluserIndice].position.x, msg->poses[rightCluserIndice].position.y);

        mtx->lock();
        KF->correct(measurement);
        mtx->unlock();
        // ROS_INFO("KF Estimation: (%f,%f)", kfEst_x, kfEst_y);
        // ROS_INFO("Measurement  : (%f,%f)", measurement.at<float>(0), measurement.at<float>(1));
    }

    return;
}