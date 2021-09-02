/*
This ROS node is used to read .pcd file and publish them on a topic to visualize them on RVIZ.
Use the launch file "read_pcd.launch"
*/

#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <my_pcl_tutorial/readConfig.h>
#include <dynamic_reconfigure/server.h>


int main (int argc, char **argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output", 1);
  
  std::string input_pcd;
  nh.getParam("input", input_pcd);
  

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  pcl::io::loadPCDFile (input_pcd , cloud); 
  pcl::toROSMsg(cloud, output);

  std::string my_frame;
  nh.getParam("frame_id", my_frame);
  output.header.frame_id=my_frame;

  ros::Rate loop_rate(1);
  while (nh.ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void callback(my_pcl_tutorial::readConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %s %s ", 
            config.frame_id.c_str(), 
            config.input.c_str()); 
           
}
