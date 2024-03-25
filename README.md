# ROB314

The project has this module but also a [controller](https://github.com/arthur-ruback/husky_controller).

### Instalation using Catkin

This repository shall be cloned into [your catktin workspace]/src.

```
git clone https://github.com/arthur-ruback/pc_pipeline
cd ..
catkin_make 
```

##### Depends on

For the correct funtioning it requires the 3D object detection module from [here](https://github.com/yzrobot/online\_learning).

### Use

##### Parameters configuration

The launchfiles defined in *launch/blablabla.launch* imports the definition of several parameters from a config file inside *config/*.

- pipeline_mode *[int]* : describes the mode of operation of this module:
    0 - VoxelGrid Filter only;
    1 - PassThrough Filter only (remove ground);
    2 - PassThrough and Voxel Filters;
    3 - PassThrough, Voxel and StatisticalOutilineRemoval Filters;

- segmentation *[bool]* : activation of segmentation of point clound into person cluster and non-person cluster.

- others...

##### Subscribed topics:
- /rslidar_points *[sensor_msgs/PointCloud2]* : lidar cloud from sensor
- /clicked_point *[geometry_msgs/PointStamped]* : person to follow from RViz
- /object3d_detector/poses *[geometry_msgs/PoseArray]* : observation pose array from detector

##### Published topics:
- /filtered_cloud *[sensor_msgs/PointCloud2]* : well filtered and adjusted cloud
- /velodyne_cloud *[sensor_msgs/PointCloud2]* : cloud with clusters only

##### Execution

The following execution is used by the authors to run simultaniously on the husky A200 robot and on the user computer. It requires a working ROS multi-machine setup.

```
[ON HUSKY] roslaunch pc_pipeline husky_no_rviz.launch
```

```
[ON PC] roslaunch pc_pipeline rviz.launch
```

You should click on the person cluster to follow to begin or to change it during execution.

