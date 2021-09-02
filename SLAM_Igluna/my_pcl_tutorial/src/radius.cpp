/*
This ROS node is used to perform a radius filter on a .pcd file https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers
It is used during the transformation from 3D map to 2D occupancy grid.
See document "SLAM PROCEDURE".
*/

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*cloud_msg, *cloud);



    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.11);
    outrem.setMinNeighborsInRadius (3);
    outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud_filtered);
 
 // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (*cloud_filtered,output);
  
// Publish the data
  pub.publish(output);

  }

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}
