/*
This ROS service is used during the transformation from the 3D map to the 2D occupancy map. 
It is used to transform the rosbag created by the grid_map_pcl into a .pcd file format.
See Document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/conversion_extraction.h"

#include <grid_map_msgs/GridMap.h>

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

bool conversion_extraction(my_pcl_tutorial::conversion_extraction::Request  &req,
             my_pcl_tutorial::conversion_extraction::Response &res)
{

// Initialization

ROS_INFO("request: file_in=%s, file_out=%s", req.file_in.c_str(), req.file_out.c_str());

//Load From Bag
const std::string pathToBag = req.file_in;
const std::string topic= "grid_map";
grid_map::GridMap gridMap;

grid_map::GridMapRosConverter::loadFromBag( pathToBag, topic, gridMap);
const std::string pointLayer ="elevation";
//Transform Grid to PC2
sensor_msgs::PointCloud2 cloud_grid_PC2;
grid_map::GridMapRosConverter::toPointCloud (gridMap, gridMap.getLayers(), pointLayer, cloud_grid_PC2);

//Partie Transform PC2 TO PCD
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grid (new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg (cloud_grid_PC2,*cloud_grid);

// Save PCD to .PCD file
pcl::PCDWriter writer;
writer.write (req.file_out, *cloud_grid, false);

  res.comment="Converting the extracted DEM map was a success";
  ROS_INFO("Sending back response: %s", res.comment.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "conversion_extraction_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("conversion_extracting", conversion_extraction);
  ROS_INFO("Ready to convert an extracted DEM map (rosbag .bag type) to a .pcd file");
  ros::spin();

  return 0;
}

