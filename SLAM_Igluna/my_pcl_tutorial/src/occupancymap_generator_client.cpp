/*
This ROS service is used to generate a 2D map grom a .pcd file.
It is used during the transformation from 3D map to 2D occupancy map. It is the last step of the process (SEE Document "SLAM_PROCEDURE).
It has several parameters :
-file_in
-file_out
-frame_id (of the generated 2D map)
-Resolution_discretized (resolution of the wanted 2D map, in meter)
-resolution_grid_map_pcl (resolution that was used by the "grid_map_pcl" node previously in the process)
It generates several files. An occupancy_grid file format is composed of two files: one .pgm and one .yaml
So there are 5 files generated :
- a Pair of .yaml and .pgm file relative to the desired grid map.
- another pair of .yaml and .pgm file relative to a discretized version of the desired grid map (can be used for path planning, task planning exploration algorithm).
- a txt file that contains the information of the grid map. It is written as a customized format (used with the ros node "map_pub.cpp")

It is the latest version that shall be used.
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_generator.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_generator");
  if (argc != 7)
  {
    ROS_INFO("usage: occupancymap_generator file_in file_out frame_id resolution resolution_discretized resolution_grid_map_pcl_node");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_generator>("occupancymap_generator");
  my_pcl_tutorial::occupancymap_generator srv;
  srv.request.file_in = argv[1];
  srv.request.file_out = argv[2];
  srv.request.frame_id=argv[3];
  srv.request.resolution = atof(argv[4]);
  srv.request.resolution_discretized = atof(argv[5]);
  srv.request.resolution_grid_map_pcl_node = atof(argv[6]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map generator was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map generator");
    return 1;
  }

  return 0;
}
