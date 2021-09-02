/*
This ROS service uses a 2D grid map (generated by "occupancy_generator" ROS service) and a database of detected objects (generated by "ar_tracker_saver.cpp")
to create the HDDL problem file used by the TASK planner algorithm.
This is the latest version that shall be used.
Several parameters are available:
-file_in_map (txt file generated by occupancy_generator or by occupancy_saver)
-file_in_artag (txt file generated by ar_tracker_saver.cpp")
-file_out
-resolution_discretized (in meter, wanted discretization for the cell of the plan)
-threshold_occupied (0.6)
-threshold_unknown (0.4)
-how_many_layers_removed (integer, if you want to remove the external border of the map because walls or something like that that shall be removed from plan)
-how_many_max_intermediary_waypoints (let's say you need to travel 3 unit distance from objective A to objective B. If this parameter is below 2,
then the travel from A to B won't be listed in the problem file. It is mainly used to reduce the complexity of the graph of the problem file, to ease the 
task planner algorithm execution)
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/occupancymap_planner.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "occupancymap_planner");
  if (argc != 9)
  {
    ROS_INFO("usage: occupancymap_planner file_in_map file_in_artag file_out resolution_discretized threshold_occupied threshold_unknown how_many_layers_removed_border how_many_max_intermediary_waypoints");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<my_pcl_tutorial::occupancymap_planner>("occupancymap_planner");
  my_pcl_tutorial::occupancymap_planner srv;
  srv.request.file_in_map = argv[1];
  srv.request.file_in_artag = argv[2];
  srv.request.file_out = argv[3];
  srv.request.resolution_discretized = atof(argv[4]);
  srv.request.thr_occ = atof(argv[5]);
  srv.request.thr_unknown = atof(argv[6]);
  srv.request.how_many_layers_removed=atoi(argv[7]);
  srv.request.how_many_max_intermediary_waypoints=atoi(argv[8]);
  if (client.call(srv))
  {
    ROS_INFO("The call of the service occupancy map planner was a success");
  }
  else
  {
    ROS_ERROR("Failed to call service occupancy map planner");
    return 1;
  }

  return 0;
}


