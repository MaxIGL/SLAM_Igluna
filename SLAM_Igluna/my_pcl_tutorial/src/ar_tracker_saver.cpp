/*
 This Algorithm consists in saving the AR tag detected by the "Ar_track_alvar" algorithm under a txt database file.
The computer vision algorithm ar_track_alvar publishes a topic which contains all the ar tag detected at the time of the received message.
This algorithm reads the already existing database to see if some new ar tag are detected (thanks to their number ID) and add them to the database if they're new.
WHAT CAN BE DONE WITH THIS ALGO:
This algorithm can be adapted to listening to any new topic containing the information of detected objects on the ground. For example, if an algorithm is developped to detect rocks,
this algorithm can be used to create the database of the number ID, position, orientation of such rocks.
WHAT CAN BE UPGRADED ON THIS ALGO:
At the present version, this algo takes into consideration only the first appearance of an object to save it in the database. This can create a lack of accuracy.
Indeed, this algorithm saves in the database only the information concerning a new detected object.
It would be better to upgrade this algorithm to save in a first database all the different coordinates of a multi-detected object, and then to compute the mean
(and discard the extremal data) relative to an object, and save those coordinates in a final database.
*/

#include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip> //used for formatting outputs
#include <sstream>
#include <vector>
#include <fstream>

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

int find_id(std::vector<int> list, int id){
for(int i=0;i!=list.size();i++){
if(id==list[i]){
return i;}
}
return -1;
}

class La_classe{
public:
    std::string filename;
    std::string topic_in;
    ros::NodeHandle _nh;
    ros::Subscriber sub;
    La_classe(ros::NodeHandle* nh):_nh(*nh)
    {
_nh.getParam("filename",this->filename);
_nh.getParam("topic_in",this->topic_in);
ROS_INFO("OK");
 sub = _nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>(topic_in, 10,&La_classe::ar_saver_cb, this);

}

void ar_saver_cb(const ar_track_alvar_msgs::AlvarMarkersConstPtr& ar_tracker)
{

//Reading data from database
        std::string ID, MOYENNE, POSITION_X, POSITION_Y, POSITION_Z, QUATERNION_X, QUATERNION_Y, QUATERNION_Z, QUATERNION_W, POSITION_CONFIDENCE; //variables from file are here
        std::vector<int>ID_v;
        std::vector<float>POSITION_X_v;
        std::vector<float>POSITION_Y_v;
        std::vector<float>POSITION_Z_v;
        std::vector<float>QUATERNION_X_v;
        std::vector<float>QUATERNION_Y_v;
        std::vector<float>QUATERNION_Z_v;
        std::vector<float>QUATERNION_W_v;
        std::vector<int>POSITION_CONFIDENCE_v;
	std::vector<int>Moyenne_v;

        //number of lines
        int number_detected=0;
        std::ifstream database(this->filename); //opening the file.
        if (database.is_open()) //if the file is open
        {
                // ROS_INFO("OPENING THE FILE FOR READING");
                //ignore first line
                std::string line;
                getline(database, line);

                while (true)
                {
                        getline(database, ID, ',');
                        if (database.eof()) break;
                        ID_v.push_back(stoi(ID));
                        getline(database, POSITION_CONFIDENCE, ',');
                        POSITION_CONFIDENCE_v.push_back(stoi(POSITION_CONFIDENCE));
                        getline(database, POSITION_X, ',');
                        POSITION_X_v.push_back(stof(POSITION_X));
                        getline(database, POSITION_Y, ',');
                        POSITION_Y_v.push_back(stof(POSITION_Y));
                        getline(database, POSITION_Z, ',');
                        POSITION_Z_v.push_back(stof(POSITION_Z));
                        getline(database, QUATERNION_X, ',');
                        QUATERNION_X_v.push_back(stof(QUATERNION_X));
                        getline(database, QUATERNION_Y, ',');
                        QUATERNION_Y_v.push_back(stof(QUATERNION_Y));
                        getline(database, QUATERNION_Z, ',');
                        QUATERNION_Z_v.push_back(stof(QUATERNION_Z));
                        getline(database, QUATERNION_W, ',');
                        QUATERNION_W_v.push_back(stof(QUATERNION_W));
			getline(database, MOYENNE, '\n');
			Moyenne_v.push_back(stoi(MOYENNE));
                        number_detected+=1;

                }
                // ROS_INFO("READING FINISHED , CLOSING THE FILE");
                database.close(); //closing the file
        }
        else ROS_INFO("Unable to open file"); //if the file is not open output



//Reading Data from current message
for (unsigned i=0;i!=ar_tracker->markers.size();i++){
int existe =find_id(ID_v,ar_tracker->markers[i].id);
if(existe!=-1){

}
if(existe==-1){
//Add the new detected marker to the database
        std::ofstream writer(this->filename,std::ios::app);
        // ROS_INFO("WRITING ON THE FILE");
        if(!writer){
        ROS_INFO("Error Opening file : ");}
        writer << ar_tracker->markers[i].id << ',' << ' ' << ar_tracker->markers[i].confidence << ',' << ' ' << ar_tracker->markers[i].pose.pose.position.x << ',' << ' ' << ar_tracker->markers[i].pose.pose.position.y << ',' << ' ' << ar_tracker->markers[i].pose.pose.position.z << ',' << ' ' << ar_tracker->markers[i].pose.pose.orientation.x << ',' << ' ' << ar_tracker->markers[i].pose.pose.orientation.y << ',' << ' ' << ar_tracker->markers[i].pose.pose.orientation.z << ',' << ' ' << ar_tracker->markers[i].pose.pose.orientation.w << 1 << '\n';
        writer.close();
//ROS_INFO("WRITING FINISHED, CLOSING FILE");
}
/*
else{
if(ar_tracker->markers[i].confidence>POSITION_CONFIDENCE_v[existe]){
//OVerwrite ici
}}
*/

}}

void run(){
        ros::Rate loop_rate(1);
         while (this->_nh.ok())
         {
           ros::spinOnce();
           loop_rate.sleep();
         }

    }


};
/*ar_ids.push_back(ar_tracker.markers[i].id);
ar_pose_confidence.push_back(ar_tracker.markers[i].confidence);
const geometry_msgs/Point ar_position_point=ar_tracker.markers[i].pose.pose.position;
std::vector<std::vector<float>> ar_position;
ar_position[0]=ar_position_point.x;
ar_position[1]=ar_position_point.y;
ar_position[2]=ar_position_point.z;
ar_positions.push_back(ar_position);
const geometry_msgs/Quaternion ar_orientation_quat=ar_tracker.markers[i].pose.pose.orientation;
std::vector<std::vector<float>> ar_orientation;
ar_orientation[0]=ar_orientation_quat.x;
ar_orientation[1]=ar_orientation_quat.y;
ar_orientation[2]=ar_orientation_quat.z;
ar_orientation[3]=ar_orientation_quat.w;
ar_orientations.push_back(ar_orientation);
}}
*/


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh("~");
  La_classe objet(&nh);
  objet.run();
}

