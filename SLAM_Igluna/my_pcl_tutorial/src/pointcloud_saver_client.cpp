/*
This ROS service is used to save a 3D map (published by octomap_server) under a .pcd file.
It is used during the transformation from 3D map to 2D occupancy grid.
See document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/pointcloud_saver.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_saver_client");
  if (argc != 3)
  {
    ROS_INFO("usage: pointcloud_saver_client topic_in file_out");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::pointcloud_saver>("pointcloud_saving");
  my_pcl_tutorial::pointcloud_saver srv;
  srv.request.topic_in = argv[1];
  srv.request.file_out = argv[2];
  if (client.call(srv))
  {
    ROS_INFO("The call of the service pointcloud_saver was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service pointcloud_saver");
    return 1;
  }

  return 0;
}
