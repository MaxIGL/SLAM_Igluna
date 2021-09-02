/*Program that takes listen to a pointcloud2 ROS topic message and try to perform a Progressive
Morphological Filter to  identify and separate ground from obstacles.
I Didn't succeed. I tried different sets of values but the algorithm didn't work. However the litterature shows great results.
You can try to make it work by trying new values.
Documentation : https://users.cs.fiu.edu/~chens/PDF/TGRS.pdf
*/

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);
  pcl::fromROSMsg (*cloud_msg, *cloud);


  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (6);
  pmf.setSlope (0.05f);
  pmf.setInitialDistance (0.25f);
  pmf.setMaxDistance (2.5f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);


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
