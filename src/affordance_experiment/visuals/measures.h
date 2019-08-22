#ifndef __OBJ_MEASURES_H__
#define __OBJ_MEASURES_H__

#include <pcl/surface/convex_hull.h>
#include <pcl/surface/impl/convex_hull.hpp>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include "types.h"
#include "geom_utils.h"

// Point cloud features
// includes: Cubeness, Sphereness, Symmetry, Convexity
// Assumes the point cloud is already downsampled with VoxelGrid and projected to its eigenbasis

// Cubeness: ratio of surface area of the object to surface area of its smallest bounding box
float scoreCubeness(PointCloud::Ptr cloud, Normals::Ptr normals) {
  
  // Find max and min of each dimension
  float maxx=cloud->points[0].x;
  float minx=cloud->points[0].x;
  float maxy=cloud->points[0].y;
  float miny=cloud->points[0].y;
  float maxz=cloud->points[0].z;
  float minz=cloud->points[0].z;
  for(int i=1;i<cloud->points.size();i++) {
    float x=cloud->points[i].x;
    if(x>maxx)maxx=x;
    if(x<minx)minx=x;
    float y=cloud->points[i].y;
    if(y>maxy)maxy=y;
    if(y<miny)miny=y;
    float z=cloud->points[i].z;
    if(z>maxz)maxz=z;
    if(z<minz)minz=z;
  }
  // Calculate smallest bounding box size
  float box_x = maxx-minx;
  float box_y = maxy-miny;
  float box_z = maxz-minz;

  // Return cubeness measure. Use number of points in point cloud to approximate surface area
  return cloud->points.size() / (2*box_x*box_y+2*box_y*box_z+2*box_z*box_x);
}

// Sphereness: ratio of surface area of the object to surface area of its smallest bounding sphere
float scoreSphereness(PointCloud::Ptr cloud) {
  
  // Find largest distance between any two points in the point cloud
  float max_dist = 0.0;
  for(int i=0;i<cloud->points.size();i++) {
    for(int j=0;j<cloud->points.size();j++) {
      if(i==j) continue;
      float x1=cloud->points[i].x;
      float y1=cloud->points[i].y;
      float z1=cloud->points[i].z;
      float x2=cloud->points[j].x;
      float y2=cloud->points[j].y;
      float z2=cloud->points[j].z;
      float dist = sqrt(pow((x1-x2),2)+pow((y1-y2),2)+pow((z1-z2),2));
      if(dist>max_dist) max_dist=dist;
    }
  }

  // Return sphereness measure. Use number of points in point cloud to approximate surface area
  return cloud->points.size() / pow(max_dist, 2);
}

// Symmetry: Reflect the cloud along all 3 principle axes and measure overlap
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
float scoreSymmetry(PointCloud::Ptr cloud, Normals::Ptr normals, float relweight) {
  
  // Get ranges of the 3 dimensions and calculate the weights
  Eigen::MatrixXf ptsmap = cloud->getMatrixXfMap();
  Eigen::VectorXf mins= ptsmap.rowwise().minCoeff();
  Eigen::VectorXf maxes= ptsmap.rowwise().maxCoeff();
  Eigen::VectorXf ranges= maxes - mins;
  Eigen::VectorXf w= ranges.head(3); // x,y,z are always the first 3 dimensions
  w /= w.sum();
  
  // Calculate symmetry
  float score=0;
  PointCloud::Ptr dest (new PointCloud);
  dest->points.resize(cloud->points.size());
  Normals::Ptr normdest (new Normals);
  normdest->points.resize(normals->points.size());
  for(int t=0;t<3;t++) {
  
    // Reflect point cloud
    for(int i=0;i<cloud->points.size();i++){
      PointT &d= dest->points[i];
      PointT &c= cloud->points[i];
      NormalT &dn= normdest->points[i];
      NormalT &cn= normals->points[i];
      if(t==0) {d.x = -c.x; dn.normal_x = -cn.normal_x;} else {d.x = c.x; dn.normal_x = cn.normal_x;}
      if(t==1) {d.y = -c.y; dn.normal_y = -cn.normal_y;} else {d.y = c.y; dn.normal_y = cn.normal_y;}
      if(t==2) {d.z = -c.z; dn.normal_z = -cn.normal_z;} else {d.z = c.z; dn.normal_y = cn.normal_y;}
    }
    // Calculate overlap
    float overlap= cloudAlignmentScoreDenseWithNormalsNormalized(cloud, normals, dest, normdest, relweight, ranges(t))
                  +cloudAlignmentScoreDenseWithNormalsNormalized(dest, normdest, cloud, normals, relweight, ranges(t));
    score += w(t)*overlap;
  }
  return -score; // return the average symmetry
}

// Convexity: Average distance of every point in the point cloud to its closest point on the convex hull
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
float scoreConvexity(PointCloud::Ptr cloud) {

  // Reconstruct convex hull
  pcl::ConvexHull<PointT> chull;
  chull.setInputCloud(cloud);
  chull.setDimension(3);
  vector<pcl::Vertices> polygons;
  PointCloud::Ptr hull (new PointCloud);
  chull.reconstruct(*hull, polygons);
  
  // make a full mesh out of it
  MeshT::Ptr m (new MeshT);
  m->polygons= polygons;
  pcl::toPCLPointCloud2(*hull, m->cloud);
  
  // subdivide the mesh
  MeshT::Ptr msub= mesh_subdivide_recursive(m, 0.005); 
  PointCloud::Ptr verts(new PointCloud);
  pcl::fromPCLPointCloud2(msub->cloud, *verts);
  
  // calculate alignment from object to hull
  float overlap= cloudAlignmentScoreDense(cloud, verts);
  return -overlap;
}



#endif