/*
This ROS service performs a outliers removal filter https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers
on an input PC2 topic.
It is used during the transformation from 3D map to 2D occupancy grid.
See document "SLAM PROCEDURE".
*/

#include "ros/ros.h"
#include "my_pcl_tutorial/outliers_removal.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

bool outliers_remove(my_pcl_tutorial::outliers_removal::Request  &req,
             my_pcl_tutorial::outliers_removal::Response &res)
{

// Initialization

ROS_INFO("request: file_in=%s, file_out=%s, radius_search=%lf, min_neighbors_in_radius=%d", req.file_in.c_str(), req.file_out.c_str(), req.radius_search, (int)req.min_neighbors_in_radius);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (req.file_in, *cloud);

pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(req.radius_search);
    outrem.setMinNeighborsInRadius (req.min_neighbors_in_radius);
    outrem.setKeepOrganized(false);
    // apply filter
    outrem.filter (*cloud_filtered);

pcl::PCDWriter writer;
  writer.write (req.file_out, *cloud_filtered, false);
  res.comment="Removing outliers was a success";
  ROS_INFO("Sending back response: %s", res.comment.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outliers_removal_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("outliers_removal", outliers_remove);
  ROS_INFO("Ready to remove the outliers of a Pointcloud (.pcd type)");
  ros::spin();

  return 0;
}
