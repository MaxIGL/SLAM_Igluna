/*
This ROS service is used during the transformation from the 3D map to the 2D occupancy map. 
It is used to transform the rosbag created by the grid_map_pcl into a .pcd file format.
See Document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/conversion_extraction.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conversion_extraction_client");
  if (argc != 3)
  {
    ROS_INFO("usage: conversion_extraction_client file_in file_out");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::conversion_extraction>("conversion_extracting");
  my_pcl_tutorial::conversion_extraction srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  if (client.call(srv))
  {
    ROS_INFO("The call of the service conversion_extraction was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service conversion_extraction");
    return 1;
  }

  return 0;
}
