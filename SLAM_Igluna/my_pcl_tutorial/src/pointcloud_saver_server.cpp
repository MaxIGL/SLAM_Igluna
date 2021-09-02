/*
This ROS service is used to save a 3D map (published by octomap_server) under a .pcd file.
It is used during the transformation from 3D map to 2D occupancy grid.
See document "SLAM PROCEDURE".
*/

#include <ros/ros.h>
#include "my_pcl_tutorial/pointcloud_saver.h"


#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

bool pointcloud_saver(my_pcl_tutorial::pointcloud_saver::Request  &req,
             my_pcl_tutorial::pointcloud_saver::Response &res)
{

ROS_INFO("request: topic_in=%s, file_out=%s", req.topic_in.c_str(),req.file_out.c_str());

sensor_msgs::PointCloud2ConstPtr pc= ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.topic_in.c_str(),ros::Duration(2));
if (pc) ROS_INFO("Map received!");
  else {
ROS_INFO("No map received! Ending.");
return true;}

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*pc, *cloud);

  pcl::PCDWriter writer;
  writer.write (req.file_out, *cloud, false);
  ROS_INFO("Point cloud saved under %s", req.file_out.c_str());

return true;
}

main (int argc, char **argv)
{
    ros::init (argc, argv, "pointcloud_saver_server");
  ros::NodeHandle n;

     ros::ServiceServer service = n.advertiseService("pointcloud_saving", pointcloud_saver);
  ROS_INFO("Ready to convert a pointcloud ros to a .pcd file");
  ros::spin();

    return 0;
}
