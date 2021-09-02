/*
This ROS service is used to save an occupancy map (possibly published by octomap_server) to a pair of .pgm + .yaml file as well as under a .txt customized file.
*/

#include <ros/ros.h>
#include "my_pcl_tutorial/occupancy_saver.h"

#include <stdio.h>
#include <stdlib.h>

#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> //used for formatting outputs
#include <sstream>
#include <vector>


bool occupancy_saver(my_pcl_tutorial::occupancy_saver::Request  &req,
             my_pcl_tutorial::occupancy_saver::Response &res)
{

ROS_INFO("request: topic_in=%s, file_out=%s", req.topic_in.c_str(),req.file_out.c_str());


nav_msgs::OccupancyGridConstPtr occupancygrid= ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(req.topic_in.c_str(),ros::Duration(2));
if (occupancygrid) ROS_INFO("Map received!");
  else {
ROS_INFO("No map received! Ending.");
return true;}

int threshold_free=25;
int threshold_occupied=65;
std::string mapname=req.file_out;


//As a pgm file 

std::string mapdatafile = mapname + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        ROS_INFO( "Couldn't save map file to %s", mapdatafile.c_str());
        return 0;
      }

      fprintf(out, "P5\n# CREATOR: Grid_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              occupancygrid->info.resolution, occupancygrid->info.width, occupancygrid->info.height);
                for(unsigned int i=0;i!=occupancygrid->info.height;i++){
                 for(unsigned int j=0;j!=occupancygrid->info.width;j++){
                 unsigned int x=j+(occupancygrid->info.height-1-i)*occupancygrid->info.width;
          if (occupancygrid->data[x] >= 0 && occupancygrid->data[x] <= threshold_free) { // Free is 254 : white
            fputc(254, out);
          } else if (occupancygrid->data[x] >= threshold_occupied) { // Occupied is black : 000
            fputc(000, out);
          } else { //unknown is 205 gray scale
            fputc(205, out);
          }
        }
        }
      fclose(out);

//As a txt file 
std::string maptxtdatafile = mapname + ".txt";
 ROS_INFO("Writing map occupancy data to %s", maptxtdatafile.c_str());
      FILE* txt = fopen(maptxtdatafile.c_str(), "w");
      if (!txt)
      {
        std::cout<< "Couldn't save map file to %s", maptxtdatafile.c_str();
        return 0;
      }

      fprintf(txt, "%.3f\n%d\n%d\n%s\n%.3f\n%.3f\n", occupancygrid->info.resolution, occupancygrid->info.width, occupancygrid->info.height, occupancygrid->header.frame_id.c_str(),occupancygrid->info.origin.position.x,occupancygrid->info.origin.position.y);
      for(unsigned int cell = 0; cell < occupancygrid->data.size(); cell++) {
       fprintf(txt, "%d ", occupancygrid->data[cell]);
      }
      fclose(txt);

//As a YAML file
      std::string mapmetadatafile = mapname + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), occupancygrid->info.resolution, occupancygrid->info.origin.position.x, occupancygrid->info.origin.position.y, 0.);

      fclose(yaml);



  ROS_INFO("Point cloud saved under %s", req.file_out.c_str());

return true;
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pointcloud_saver_server");
  ros::NodeHandle n;

     ros::ServiceServer service = n.advertiseService("occupancy_saving", occupancy_saver);
  ROS_INFO("Ready to save an occupancy map to .pgm .yaml and .txt files");
  ros::spin();

    return 0;
}
