/*
This script is used to denoise a .pcd file.
To use it you have ot manually change the file address in this file.
A better version (a ros node for denoising an input topic or a ros service for densoising a single file) shall be used to perform this task.
*/

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <pcl/common/centroid.h>




int main(int argc, char** argv)
{

// Initialization
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// Fill in the cloud data
pcl::PCDReader reader;
reader.read ("/home/dcas/m.dreier/Documents/PCD_FILES/Test3/Laser/Outside_banc_radiusx1_denoisexdx1_grid.pcd", *cloud);

pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

kdtree.setInputCloud (cloud);


// Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
double radius=0.1;
double epsilon=0.1;

int compt=0;
for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin(); it != cloud->end(); it++) {
pcl::PointXYZ search_point;
search_point.x=it->x;
search_point.y=it->y;
search_point.z=it->z;
compt+=1;
  if ( kdtree.radiusSearch (search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){


pcl::PointCloud<pcl::PointXYZ> neighbors;

for (std::size_t y = 0; y < pointIdxRadiusSearch.size (); ++y){
neighbors.push_back((*cloud)[pointIdxRadiusSearch[y]]);}

// Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;
  Eigen::Vector3f xyz_cent;

  // Estimate the XYZ centroid
  compute3DCentroid (neighbors, xyz_centroid);
  for(int z=0;z<3;z++) xyz_cent(z)=xyz_centroid(z);
  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (neighbors, xyz_centroid, covariance_matrix);
  Eigen::Matrix3f mat;
mat(0,0),mat(1,1),mat(2,2)=epsilon;
mat(1,0),mat(1,2),mat(2,0),mat(2,1),mat(0,1),mat(0,2)=0;
mat+=covariance_matrix;
  Eigen::Matrix3f inverse;
//Compute the inverse
double determinant;
//Detrminant
	for(int k = 0; k < 3; k++){
		determinant = determinant + (mat(0,k) * (mat(1,(k+1)%3) * mat(2,(k+2)%3) - mat(1,(k+2)%3) * mat(2,(k+1)%3)));}

	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			inverse(i,j)=((mat((j+1)%3,(i+1)%3) * mat((j+2)%3,(i+2)%3)) - (mat((j+1)%3,(i+2)%3) * mat((j+2)%3,(i+1)%3)))/determinant;}
	}
Eigen::Matrix3f A;
A=covariance_matrix*inverse;
Eigen::Vector3f B;
B=xyz_cent-A*xyz_cent;
Eigen::Vector3f e_pt;
e_pt(0)=search_point.x;
e_pt(1)=search_point.y;
e_pt(2)=search_point.z;
Eigen::Vector3f A_pt;
A_pt=A*e_pt+B;
pcl::PointXYZ pt;
pt.x=A_pt(0);
pt.y=A_pt(1);
pt.z=A_pt(2);
(*cloud_filtered).push_back(pt);
  }
}

//Save the output cloud_filtered
pcl::PCDWriter writer;
  writer.write ("/home/dcas/m.dreier/Documents/PCD_FILES/Test3/Laser/Outside_banc_radiusx1_denoisexdx1_grid_denoisedx1.pcd", *cloud_filtered, false);

return(0);
}

