# rob314

***!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!***

***Primitive version, still needs to manually select the cloud to follow***

***!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!***

The hole project has this module but also a [insert link] controller and a [insert link] SLAM.

### Instalation using Catkin

This repository shall be cloned into [your catktin workspace]/src.

```
git clone https://github.com/arthur-ruback/pc_pipeline
cd ..
catkin_make 
```

### Use

##### Parameters configuration

The launchfile defined in *launch/pipeline.launch* contains, other thant the launch of this module and RViz, the definition of several parameters.

- pipeline_mode *[int]* : describes the mode of operation of this module:
    0 - VoxelGrid Filter only;
    1 - PassThrough Filter only (remove ground);
    2 - PassThrough and Voxel Filters;
    3 - PassThrough, Voxel and StatisticalOutilineRemoval Filters;

- segmentation *[bool]* : activation of segmentation of point clound into person cluster and non-person cluster.

- others...

##### Subscribed topics:
- /rslidar_points *[sensor_msgs/PointCloud2]*
- /clicked_point *[geometry_msgs/PointStamped]*

##### Published topics:
- /filtered_cloud *[sensor_msgs/PointCloud2]*
- /cluster *[sensor_msgs/PointCloud2]*

##### Execution

```
roslaunch pc_pipeline pipeline.launch
```

### TODOs:
- select automatically the person from the cloud in the first iteraction and get it as first measure;
- verify the cases where the pointcloud is lost or too different (re-analize all the cloud);
- fix the XY inversion in frame, specially with the robot moving frame
- change the KF to take into account the movement of the robot
- publish the pointCloud without the person

