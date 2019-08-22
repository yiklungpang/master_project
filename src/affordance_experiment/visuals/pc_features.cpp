#include <iostream>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include "types.h"
#include "geom_utils.h"
#include "measures.h"

// Compute visual features from point clouds
// Read the point clouds from PCD files, compute the visual features then
// output the feature measures to a CSV file.

// Load the PCD file and return the point cloud
void findPCD(PointCloud::Ptr cloud, const std::string object_name, const std::string position)
{
  // Get path to data directory
  char cwd[1024];
  string current_path = getcwd(cwd, sizeof(cwd));
  std::size_t pos = current_path.find("/build");
  string dir = string(current_path.substr(0,pos)+"/data/");

  // Look for PCD file of object & position in the data directory
  vector<string> files = vector<string>();
  DIR *dp;
  struct dirent *ep;
  bool filefound = false;
  dp = opendir(dir.c_str());
  if (dp != NULL){
    while ((ep = readdir (dp)) != NULL){
      if (filefound == true){
        break;
      }
      string fname = string(ep->d_name);
      if (fname.find(string(object_name+"_"+position)) == 0) {
        // Load PCD file
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (string(dir+fname), *cloud) == -1)
        {
          PCL_ERROR ("Couldn't read pcd file \n");
          exit(0);
        } else {
          filefound = true;
          cout << "PCD file found " << string(fname) << "\n";
        }
      }
    }
    closedir(dp);
    if (filefound == false) {
      PCL_ERROR ("Couldn't find pcd file for object %s in position %s \n", string(object_name), string(position));
    }
  }
}


// Perform object segmentation by transforming the point cloud to the world frame and applying a height filter
void segmentPCD (PointCloud::Ptr cloud_filtered, const std::string object_name, const std::string position)
{
  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr transformed_cloud1 (new PointCloud);
  PointCloud::Ptr transformed_cloud2 (new PointCloud);
  PointCloud::Ptr transformed_cloud3 (new PointCloud);
  PointCloud::Ptr transformed_cloud4 (new PointCloud);

  // Load point cloud from PCD file
  findPCD (cloud, object_name, position);

  // Transform to match world frame and centre on object in z-axis
  // Rotate along X axis
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
  float theta1 = -2.75762;
  transform_1.rotate (Eigen::AngleAxisf (theta1, Eigen::Vector3f::UnitX()));
  pcl::transformPointCloud (*cloud, *transformed_cloud1, transform_1);
  // Translate along Y and Z axis
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
  transform_2.translation() << 0.0, -0.189, 1.6565;
  pcl::transformPointCloud (*transformed_cloud1, *transformed_cloud2, transform_2);

  // Rotate based on recorded pose
  if (position != "front") {
    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    float theta3 = 0.0;
    if (position == "left") {
      theta3 = -1.570796327;
    } else if (position == "back") {
      theta3 = -3.141592654;
    } else if (position == "right") {
      theta3 = -4.71238898;
    }

    transform_3.rotate (Eigen::AngleAxisf (theta3, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*transformed_cloud2, *transformed_cloud3, transform_3);
  }

  // Discard background (anything below the tabletop)
  pcl::PassThrough<pcl::PointXYZ> pass;
  if (position == "front") {
    pass.setInputCloud (transformed_cloud2);
  } else {
    pass.setInputCloud (transformed_cloud3);
  }
  
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.996, 2.0);
  pass.filter (*cloud_filtered);

}


// Remove the handle part of the tool
// Find range in y axis and remove half of the point cloud
void removeHandle (PointCloud::Ptr cloud) {
  // Find min and max in y axis
  float maxy=cloud->points[0].y;
  float miny=cloud->points[0].y;
  for(int i=1;i<cloud->points.size();i++) {
    float y=cloud->points[i].y;
    if(y>maxy)maxy=y;
    if(y<miny)miny=y;
  }

  // Remove half of the point cloud
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (miny, (maxy-miny)/2.0+miny);
  pass.filter (*cloud);
}


// Main function of the program
int main (int argc, char** argv)
{
  // Get path to output directory
  char cwd[1024];
  string current_path = getcwd(cwd, sizeof(cwd));
  std::size_t pos = current_path.find("/visuals");

  // Get path to data directory
  string dir = string(current_path.substr(0,pos)+"/bn/features/pc_features.csv");

  // Output features to csv file
  std::ofstream output_file(dir.c_str());
  // output_file.open ("pc_features.csv");
  // Header line
  output_file << "object_name,sphereness,cubeness,symmetry,convexity,eccentricity\n";

  // DEBUG_MODE
  bool DEBUG_MODE = false;
  if (argc == 2 && string(argv[1]) == "-d") {
    DEBUG_MODE = true;
  }

  // List of objects to be processed
  vector<string> object_list = vector<string>();
  object_list.push_back("cube");
  object_list.push_back("sphere");
  object_list.push_back("cylinder");
  object_list.push_back("stick");
  object_list.push_back("l_stick");
  object_list.push_back("bone");
  object_list.push_back("umbrella");
  object_list.push_back("fork");

  // List of tools (requires removal of handle)
  vector<string> tool_list = vector<string>();
  tool_list.push_back("stick");
  tool_list.push_back("l_stick");
  tool_list.push_back("bone");
  tool_list.push_back("umbrella");
  tool_list.push_back("fork");

  // Calculate features for each object and output to csv
  int n_obj = object_list.size();
  for (int j=0; j<n_obj; j++) {

    PointCloud::Ptr cloud_filtered_front (new PointCloud);
    PointCloud::Ptr cloud_filtered_left (new PointCloud);
    PointCloud::Ptr cloud_filtered_right (new PointCloud);  
    PointCloud::Ptr cloud_filtered_back (new PointCloud);  

    PointCloud::Ptr cloud_filtered_combined (new PointCloud);
    PointCloud::Ptr cloud_filtered_final (new PointCloud);
    Normals::Ptr normals_filtered_final (new Normals);  
    PointCloudWithNormals::Ptr point_cloud_normals (new PointCloudWithNormals);

    PointCloud::Ptr proj_cloud (new PointCloud);
    Normals::Ptr proj_normals (new Normals);  

    // Read and segment the object point clouds from PCD
    string object_name = object_list[j];
    output_file << object_name << ",";
    segmentPCD (cloud_filtered_front, object_name, "front");
    segmentPCD (cloud_filtered_left, object_name, "left");
    segmentPCD (cloud_filtered_right, object_name, "right");
    segmentPCD (cloud_filtered_back, object_name, "back");

    if (DEBUG_MODE == true) {
      PCL_INFO("No. of points in point clouds:");
      PCL_INFO("Front: %i \n", cloud_filtered_front->points.size());
      PCL_INFO("Left: %i \n", cloud_filtered_left->points.size());
      PCL_INFO("Right: %i \n", cloud_filtered_right->points.size());
      PCL_INFO("Back: %i \n", cloud_filtered_back->points.size());
    }

    // Combine the point clouds of different pose
    *cloud_filtered_combined += *cloud_filtered_front;
    *cloud_filtered_combined += *cloud_filtered_left;
    *cloud_filtered_combined += *cloud_filtered_back;
    *cloud_filtered_combined += *cloud_filtered_right;

    // Downsample to uniform point cloud
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize (0.0025, 0.0025, 0.0025);
    grid.setInputCloud (cloud_filtered_combined);
    grid.filter (*cloud_filtered_final);
    cout << "Before downsample: " << cloud_filtered_combined->points.size() << " After downsample: " << cloud_filtered_final->points.size() << "\n";

    // Remove tool handle
    if (std::find(tool_list.begin(), tool_list.end(), object_name) != tool_list.end()) {
      removeHandle (cloud_filtered_final);
    }

    if (DEBUG_MODE == true) {
      pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
      viewer.showCloud (cloud_filtered_final);
      while (!viewer.wasStopped ())
      {
      }
    }

    // Compute normals and perform PCA
    computeNormal (cloud_filtered_final, normals_filtered_final);
    float lambda0n, lambda1n, lambda2n;
    projectToEigenbasis (cloud_filtered_final, normals_filtered_final, proj_cloud, proj_normals, lambda0n, lambda1n, lambda2n);
    
    // Compute features and output to csv
    output_file << fixed;

    float sphereness;
    sphereness = scoreSphereness(proj_cloud);
    output_file << sphereness << ",";

    float cubeness;
    cubeness = scoreCubeness(proj_cloud, proj_normals);
    output_file << cubeness << ",";

    float symmetry;
    symmetry = scoreSymmetry(proj_cloud, proj_normals, 0.2);
    output_file << symmetry << ",";

    float convexity;
    convexity = scoreConvexity(proj_cloud);
    output_file << convexity << ",";

    float eccentricity;
    eccentricity = ((lambda1n/lambda0n)+(lambda2n/lambda0n))/2.0;
    output_file << eccentricity << "\n";

    cout << "Processed " << object_name << "\n";
    if (DEBUG_MODE == true) {
      PCL_INFO("Sphereness. %lf\n", sphereness);
      PCL_INFO("Cubeness. %lf\n", cubeness);
      PCL_INFO("Symmetry. %lf\n", symmetry);
      PCL_INFO("Convexity. %lf\n", convexity);
      PCL_INFO("Eccentricity. %lf\n", eccentricity);
    }

  }
}