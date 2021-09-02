/*
This ROS service performs a outliers removal filter https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers
on an input PC2 topic.
It is used during the transformation from 3D map to 2D occupancy grid.
See document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/outliers_removal.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outliers_removal_client");
  if (argc != 5)
  {
    ROS_INFO("usage: outliers_removal_client file_in file_out radius_search min_neighbors_in_radius");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::outliers_removal>("outliers_removal");
  my_pcl_tutorial::outliers_removal srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  srv.request.radius_search = atof(argv[3]);
  srv.request.min_neighbors_in_radius = atoi(argv[4]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service outliers_removal was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service outliers_removal");
    return 1;
  }

  return 0;
}

