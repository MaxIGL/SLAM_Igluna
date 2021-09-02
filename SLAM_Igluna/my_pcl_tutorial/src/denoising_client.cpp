/*
This ROS service is used to denoise a .pcd file.
It is used during the transformation between the 3D map and 2D occupancy map.
See document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/denoising.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "denoising_client");
  if (argc != 3)
  {
    ROS_INFO("usage: denoising_client file_in file_out");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::denoising>("denoising");
  my_pcl_tutorial::denoising srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  if (client.call(srv))
  {
    ROS_INFO("The call of the service denoising was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service denoising");
    return 1;
  }

  return 0;
}

