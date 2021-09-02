/*
This ROS service is used to save an occupancy map (possibly published by octomap_server) to a pair of .pgm + .yaml file as well as under a .txt customized file.
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancy_saver.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancy_saver_client");
  if (argc != 3)
  {
    ROS_INFO("usage: occupancy_saver_client topic_in file_out");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancy_saver>("occupancy_saving");
  my_pcl_tutorial::occupancy_saver srv;
  srv.request.topic_in = argv[1];
  srv.request.file_out = argv[2];
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy_saver was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy_saver");
    return 1;
  }

  return 0;
}
