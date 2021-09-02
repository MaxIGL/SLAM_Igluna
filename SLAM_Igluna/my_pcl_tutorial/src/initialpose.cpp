/*
This Ros Node listens to the TFs in order to see if a TF has been published by "ar_tf.cpp". If so, an initial position for the rover on the map can be computed and
be displayed under a topic "initial_pose" and can be used by the path_planning team.
*/

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <sstream>


std::string child_frame;
std::string parent_frame;
geometry_msgs::PoseWithCovarianceStamped initial_pose;
void listenerCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  initial_pose.header.stamp=ros::Time::now();
  initial_pose.header.seq=0;
  initial_pose.header.frame_id=parent_frame;
  for (int i=0;i!=msg->transforms.size();i++){
  if (msg->transforms[i].child_frame_id==child_frame){
initial_pose.header=msg->transforms[i].header;
initial_pose.pose.pose.position.x=msg->transforms[i].transform.translation.x;
initial_pose.pose.pose.position.y=msg->transforms[i].transform.translation.y;
initial_pose.pose.pose.position.z=msg->transforms[i].transform.translation.z;
initial_pose.pose.pose.orientation.x=msg->transforms[i].transform.rotation.x;
initial_pose.pose.pose.orientation.y=msg->transforms[i].transform.rotation.y;
initial_pose.pose.pose.orientation.z=msg->transforms[i].transform.rotation.z;
initial_pose.pose.pose.orientation.w=msg->transforms[i].transform.rotation.w;
}

}
}



int main(int argc, char **argv)
{
for (int i=0;i!=36;i++){
 initial_pose.pose.covariance[i]=0;}
initial_pose.pose.pose.position.x=0;
initial_pose.pose.pose.position.y=0;
initial_pose.pose.pose.position.z=0;
initial_pose.pose.pose.orientation.x=0;
initial_pose.pose.pose.orientation.y=0;
initial_pose.pose.pose.orientation.z=0;
initial_pose.pose.pose.orientation.w=1;

 ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");
std::string topic_out;
std::string topic_in;
nh.getParam("topic_out",topic_out);
nh.getParam("topic_in",topic_in);
nh.getParam("child_frame",child_frame);
nh.getParam("parent_frame",parent_frame);
  ros::Subscriber sub=nh.subscribe(topic_in, 1,listenerCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_out, 1);
  ros::Rate loop_rate(0.1);
  while (ros::ok())
  {
    
    ros::spinOnce();
pub.publish(initial_pose);
    loop_rate.sleep();
  }


  return 0;
}
