#include <ros/ros.h>
#include "pc_pipeline/pc_pipeline_treatment.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pc_pipeline_node");
  ros::NodeHandle nodeHandle("~");

  myPipeline myPipeline(nodeHandle);

  ros::spin();
  return 0;
}
