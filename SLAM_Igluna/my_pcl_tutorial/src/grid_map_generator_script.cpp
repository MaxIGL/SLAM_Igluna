/*
This script is used to create a 2D occupancy map out of a .pcd file (3D pcd).
This is an old version. The ROS service shall be used instead.
See Document "SLAM PROCEDURE".
*/

#include <cmath>
#include <vector>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>

#include <nav_msgs/OccupancyGrid.h>
#include "yaml-cpp/yaml.h"
#include <fstream>

int main(int argc, char** argv)
{
//Initialization
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
reader.read ("/home/dcas/m.dreier/Documents/PCD_FILES/Test2/D435/gridmappclDenoisedx1.pcd", *cloud);
int width;
int height;
double origin_x;
double origin_y;
double resolution=0.06;

//Get Min and Max x and y and compute width and height of the 2D map
float min_x = cloud->points[0].x, min_y = cloud->points[0].y, max_x = cloud->points[0].x, max_y = cloud->points[0].y,min_z=cloud->points[0].z,max_z=cloud->points[0].z;
for (size_t i = 1; i < cloud->points.size (); ++i){
       if(cloud->points[i].x <= min_x )
           min_x = cloud->points[i].x;
       if(cloud->points[i].y <= min_y )
           min_y = cloud->points[i].y;
       if(cloud->points[i].z <= min_z )
           min_z = cloud->points[i].z;
       if(cloud->points[i].x >= max_x )
           max_x = cloud->points[i].x;
        if(cloud->points[i].y >= max_y )
           max_y = cloud->points[i].y;
        if(cloud->points[i].z >= max_z )
           max_z = cloud->points[i].z;
   }
origin_x=min_x;
origin_y=min_y;
//Dimensions in meter
float width_float  = max_x - min_x;
float height_float = max_y - min_y;
//Number of cells necessary
float width_scaled = width_float  / resolution - 1/2;
float height_scaled= height_float / resolution - 1/2;
width  = int(width_scaled)+2;
height = int(height_scaled)+2; //Truncation to the lower int means we need to add 1 cell, and we want origin point to fall in the middle of the cell so we need to add 1 another cell. 
std::cout<< "width ";
std::cout<< width; 
std::cout<<" height "; 
std::cout<<height;std::cout<< " origine x "; std::cout<<min_x;std::cout << "lala max_x"; std::cout<< max_x;std::cout<< "  oki max_y "; std::cout << max_y; std::cout<<" origine y ";std::cout<<min_y;std::cout<< "min_z " ;std::cout<<min_z; std::cout << " max_z "; std::cout << max_z;

//Instantiate 2.5D map
std::vector<std::vector<float>> twofiveD_depth(width*height);

//iteration on the point cloud to fill the 2D map

for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
pcl::PointXYZ search_point;
search_point.x=it->x;
search_point.y=it->y;
search_point.z=it->z;
int i,j;
//Colonne
float cell_x= (search_point.x-origin_x)/resolution -1/2; //normalement compris entre k-1 et k
if (cell_x<=0) j=0;
else j= int(cell_x)+1;
//Ligne
float cell_y= (search_point.y-origin_y)/resolution -1/2; //normalement compris entre k-1 et k
if (cell_y<=0) i=0;
else i= int(cell_y)+1;

if(min_z <=0){
twofiveD_depth[i*width+j].push_back(search_point.z-min_z);

} //if some depths are negative, translate map to positive values
else {twofiveD_depth[i*width+j].push_back(search_point.z);
}
}

std::vector<float> twoD_depth(width*height);


//Assume that the 2.5D map is 1 layer thick. Further work later
//std::vector<std::string> trust(width*height); //"A" for valid, "B" for invalid
//std::vector<float> twoD_depth(width*height);
//Iteration sur la 2.5D map created 
//for (int num=0;num!=twofiveD_depth.size();num++){
//if twofiveD_depth[num].size()==0 twoD_depth[num]=-1; //The cell is unknown 
//float moyenne=0;

//std::sort (twofiveD_depth[num].begin(),twofiveD_depth[num].end());

//We compute the mean value of the column
//for (Iter=twofiveD_depth[num].begin(); Iter!=twofiveD_depth[num].end();Iter++){ 
//moyenne+=Iter;


//}
//moyenne=moyenne/twofiveD_depth[num].size;

//We detect if there are some aberrant values in the column
//std::string trust_num="A";
//for (Iter=twofiveD_depth[num].begin(); Iter!=twofiveD_depth[num].end();Iter++){
//if (Iter<=moyenne){
//if(moyenne-Iter >= 0.02){
//trust_num="B";}}
//else if (Iter >=moyenne){
//if (Iter-moyenne >=0.02){
//trust_num="B";}}

//if (trust_num=="B"){

// , 
//twoD_depth[num]=moyenne;
//}
//


//Convert 25D to 2D depth
for (size_t num=0;num!=twofiveD_depth.size();num++){

if (twofiveD_depth[num].size()==0) twoD_depth[num]=-1; //The cell is unknown 
else if(twofiveD_depth[num].size()!=0){
std::sort (twofiveD_depth[num].begin(),twofiveD_depth[num].end(),std::greater<float>());
float moyenne=0;
int compteur=0;
float precedent=twofiveD_depth[num][0];
for (size_t Iter=0; Iter!=twofiveD_depth[num].size();Iter++){

if (precedent-twofiveD_depth[num][Iter]>=0.04){
std::cout<< "Alerte au gogol !";}

//if (precedent-twofiveD_depth[num][Iter]<=0.04){
moyenne+=twofiveD_depth[num][Iter];
precedent=twofiveD_depth[num][Iter];
compteur+=1;
}

twoD_depth[num]=moyenne/compteur;
//We compute the mean value of the column
}}

// compute the depth gradient and register obstacles into 2D list
std::vector<int8_t> occupancygridlist(width*height);

//Iteration on horizontal neighbors
for (int j=0;j!=width-1;j++){
for (int i=0;i!=height;i++){

//Check if the cell is unknown before computing depth gradient
if (twoD_depth[j+width*i]==-1) occupancygridlist[j+width*i]=-1;
if (twoD_depth[j+width*i+1]==-1) occupancygridlist[j+1+width*i]=-1;
if (twoD_depth[j+width*i]!=-1 and twoD_depth[j+1+width*i]!=-1){
if(fabs(twoD_depth[j+width*i]-twoD_depth[j+1+width*i])>=0.075){ //Determine if the cell is the cell of an obstacle
if(twoD_depth[j+width*i]>=twoD_depth[j+1+width*i]){
occupancygridlist[j+width*i]=100;
}
else{

occupancygridlist[j+1+width*i]=100;} //Add obstacle and only obstacle on the grid
}}}}

//Iteration on vertical neighbors
for (int j=0;j!=width;j++){
for (int i=0;i!=height-1;i++){
//Check if the cell is unknown before computing depth gradient
if (twoD_depth[j+width*i]==-1) occupancygridlist[j+width*i]=-1;
if (twoD_depth[j+width*(i+1)]==-1) occupancygridlist[j+width*(i+1)]=-1;
if (twoD_depth[j+width*i]!=-1 and twoD_depth[j+width*(i+1)]!=-1){
if(fabs(twoD_depth[j+width*i]-twoD_depth[j+width*(i+1)])>=0.075){ //Determine if the cell is the cell of an obstacle
if(twoD_depth[j+width*i]>=twoD_depth[j+width*(i+1)]){
occupancygridlist[j+width*i]=100;
}
else{

occupancygridlist[j+width*(i+1)]=100;} //Add obstacle and only obstacle on the grid
}}}}

//Iteration on diagonials
for (int j=0;j!=width-1;j++){
for (int i=1;i!=height;i++){
//Check if the cell is unknown before computing depth gradient
if (twoD_depth[j+width*i]==-1) occupancygridlist[j+width*i]=-1;
 if (twoD_depth[j+1+width*(i-1)]==-1) occupancygridlist[j+1+width*(i-1)]=-1;
 if (twoD_depth[j+width*i]!=-1 and twoD_depth[j+1+width*(i-1)]!=-1){
if(fabs(twoD_depth[j+width*i]-twoD_depth[j+1+width*(i-1)])>=0.075){ //Determine if the cell is the cell of an obstacle
if(twoD_depth[j+width*i]>=twoD_depth[j+1+width*(i-1)]){
occupancygridlist[j+width*i]=100;
}
else{

occupancygridlist[j+1+width*(i-1)]=100;} //Add obstacle and only obstacle on the grid
}}}}

//Iteration on anti-diagonals
for (int j=0;j!=width-1;j++){
for (int i=0;i!=height-1;i++){
//Check if the cell is unknown before computing depth gradient
if (twoD_depth[j+width*i]==-1) occupancygridlist[j+width*i]=-1;
if (twoD_depth[j+1+width*(i+1)]==-1) occupancygridlist[j+1+width*(i+1)]=-1;
if (twoD_depth[j+width*i]!=-1 and twoD_depth[j+1+width*(i+1)]!=-1){
if(fabs(twoD_depth[j+width*i]-twoD_depth[j+1+width*(i+1)])>=0.075){ //Determine if the cell is the cell of an obstacle
if(twoD_depth[j+width*i]>=twoD_depth[j+1+width*(i+1)]){
occupancygridlist[j+width*i]=100;
}
else{

occupancygridlist[j+1+width*(i+1)]=100;} //Add obstacle and only obstacle on the grid
}}}}

//Create the final sensor::msg occupancygrid 2D occupancy map

nav_msgs::OccupancyGrid occupancygrid;
occupancygrid.header.frame_id="map";
occupancygrid.info.resolution=resolution;
occupancygrid.info.width=width;
occupancygrid.info.height=height;
occupancygrid.info.origin.position.x=origin_x;
occupancygrid.info.origin.position.y=origin_y;
occupancygrid.info.origin.orientation.x = 0.0;
occupancygrid.info.origin.orientation.y = 0.0;
occupancygrid.info.origin.orientation.z = 0.0;
occupancygrid.info.origin.orientation.w = 1.0;
occupancygrid.data=occupancygridlist;


//Code for Jasmine map
double res_discret=0.1;
int cotes= res_discret/resolution+1;
int rectangle_large=width/cotes;
int restant_large=width-rectangle_large*cotes;
int rectangle_haut=height/cotes;
int restant_haut=height-rectangle_haut*cotes;
if(restant_haut!=0){
rectangle_haut+=1;}
if (restant_large!=0){
rectangle_large+=1;}
std::vector<std::vector<int>> mapdiscret(rectangle_large*rectangle_haut, std::vector<int>(3));
std::cout<< "resolution ";
std::cout<< cotes*resolution;
std::cout<< " width : ";
std::cout<< rectangle_large;
std::cout<< " height : ";
std::cout<< rectangle_haut;

for (unsigned i=0;i!=height;i++){
for(unsigned j=0;j!=width;j++){
int x=i/cotes;
int y=j/cotes;
mapdiscret[y+x*rectangle_large][0]+=1;
if (occupancygridlist[j+width*i]==100) mapdiscret[y+x*rectangle_large][1]+=1;
if(occupancygridlist[j+width*i]==-1) mapdiscret[y+x*rectangle_large][2]+=1;
}}

std::vector<int8_t> griddiscret(rectangle_large*rectangle_haut);
for (unsigned i=0; i!=rectangle_large*rectangle_haut;i++){
double percentage_occ;
double percentage_unknown;
if (mapdiscret[i][2]!=mapdiscret[i][0]){
percentage_occ=mapdiscret[i][1]/double(mapdiscret[i][0]-mapdiscret[i][2]);
percentage_unknown=mapdiscret[i][2]/double(mapdiscret[i][0]);}
else {
percentage_occ=0;
percentage_unknown=1;}
if (percentage_occ>=0.05) griddiscret[i]=100;
else if(percentage_unknown>=0.4) griddiscret[i]=-1;}



nav_msgs::OccupancyGrid occupancygriddiscret;
occupancygriddiscret.header.frame_id="map";
occupancygriddiscret.info.resolution=cotes*resolution;
occupancygriddiscret.info.width=rectangle_large;
occupancygriddiscret.info.height=rectangle_haut;
occupancygriddiscret.info.origin.position.x=origin_x;
occupancygriddiscret.info.origin.position.y=origin_y;
occupancygriddiscret.info.origin.orientation.x = 0.0;
occupancygriddiscret.info.origin.orientation.y = 0.0;
occupancygriddiscret.info.origin.orientation.z = 0.0;
occupancygriddiscret.info.origin.orientation.w = 1.0;
occupancygriddiscret.data=griddiscret;

//Save the map to a file
std::string mapname="/home/dcas/m.dreier/Documents/FinalGrid";
int threshold_free=25;
int threshold_occupied=65;

//As a pgm file 

std::string mapdatafile_discret = mapname + "_discret.pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile_discret.c_str());
      FILE* out_discret = fopen(mapdatafile_discret.c_str(), "w");
      if (!out_discret)
      {
        std::cout<< "Couldn't save map file to %s", mapdatafile_discret.c_str();
        return 0;
      }

      fprintf(out_discret, "P5\n# CREATOR: Grid_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              occupancygriddiscret.info.resolution, occupancygriddiscret.info.width, occupancygriddiscret.info.height);
	for(unsigned int i=0;i!=occupancygriddiscret.info.height*occupancygriddiscret.info.width;i++){
          if (occupancygriddiscret.data[i] >= 0 && occupancygriddiscret.data[i] <= threshold_free) { // Free is 254 : white
            fputc(254, out_discret);
          } else if (occupancygriddiscret.data[i] >= threshold_occupied) { // Occupied is black : 000
            fputc(000, out_discret);
          } else { //unknown is 205 gray scale
            fputc(205, out_discret);
          }
        }
      fclose(out_discret);


      std::string mapmetadatafile_discret = mapname + "_discret.yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile_discret.c_str());
      FILE* yaml_discret = fopen(mapmetadatafile_discret.c_str(), "w");


      fprintf(yaml_discret, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile_discret.c_str(), occupancygriddiscret.info.resolution, occupancygriddiscret.info.origin.position.x, occupancygriddiscret.info.origin.position.y, 0.);

      fclose(yaml_discret);

/*
//Save the map to a file
std::string mapname="/home/dcas/m.dreier/Documents/FinalGrid";
int threshold_free=25;
int threshold_occupied=65;
*/

//As a pgm file 

std::string mapdatafile = mapname + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out)
      {
        std::cout<< "Couldn't save map file to %s", mapdatafile.c_str();
        return 0;
      }

      fprintf(out, "P5\n# CREATOR: Grid_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
              occupancygrid.info.resolution, occupancygrid.info.width, occupancygrid.info.height);
	for(unsigned int i=0;i!=occupancygrid.info.height*occupancygrid.info.width;i++){
          if (occupancygrid.data[i] >= 0 && occupancygrid.data[i] <= threshold_free) { // Free is 254 : white
            fputc(254, out);
          } else if (occupancygrid.data[i] >= threshold_occupied) { // Occupied is black : 000
            fputc(000, out);
          } else { //unknown is 205 gray scale
            fputc(205, out);
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

      fprintf(txt, "P5\n# CREATOR: Grid_map_generator.cpp \n%d %d\n255\n",
               occupancygrid.info.width, occupancygrid.info.height);
      for(unsigned int y = 0; y < occupancygrid.info.height; y++) {
        for(unsigned int x = 0; x < occupancygrid.info.width; x++) {
          unsigned int i = x + (occupancygrid.info.height - y - 1) * occupancygrid.info.width;
if ((x+y)%15==0) fprintf(txt,"\n");
          if (occupancygrid.data[i] >= 0 && occupancygrid.data[i] <= threshold_free) { // Free is 254 : white
            fprintf( txt,"%d ", 254);
          } else if (occupancygrid.data[i] >= threshold_occupied) { // Occupied is black : 000
            fprintf( txt, "%d ",000);
          } else { //unknown is 205 gray scale
            fprintf( txt, "%d ", 205);
          }
        }
      }
      fclose(txt);


      std::string mapmetadatafile = mapname + ".yaml";
      ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
      FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), occupancygrid.info.resolution, occupancygrid.info.origin.position.x, occupancygrid.info.origin.position.y, 0.);

      fclose(yaml);
 
return 0;
}
