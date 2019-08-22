#ifndef __OBJ_GEOM_UTILS_H__
#define __OBJ_GEOM_UTILS_H__

#include "types.h"
#include <vector>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace Eigen;

// Utility functions

// Compute the normal of a point cloud
// This is modified from:
// *Source*: http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation
void computeNormal(PointCloud::Ptr cloud, Normals::Ptr &normals) {

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointT, PointNormalT> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    // Use k neighbors
    ne.setKSearch (10);

    // Output datasets
    PointCloudWithNormals::Ptr cn (new PointCloudWithNormals); 
    Normals::Ptr n(new Normals);

    // Compute the normals
    ne.compute (*cn);
    int cn_size = cn->points.size();
    n->points.resize(cn_size);
    for(int j=0; j<cn_size; j++) {
      n->points[j].normal_x = cn->points[j].normal_x;
      n->points[j].normal_y = cn->points[j].normal_y;
      n->points[j].normal_z = cn->points[j].normal_z;
      n->points[j].curvature = cn->points[j].curvature;
    }
    normals = n;
}

// takes cloud, normals and projects them to eigenbasis
// returns result in projcloud, projnormals, 
// and the lambda1normalized and lambda2normalized are second and third eigenvalues of the scatter matrix 
// divided by the first. (so lambda2 < lambda1)
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
void projectToEigenbasis(PointCloud::Ptr cloud, Normals::Ptr normals, PointCloud::Ptr &projcloud, Normals::Ptr &projnormals, float &lambda0normalized, float &lambda1normalized, float &lambda2normalized) {
  
    pcl::PCA<PointT> pca;
    pca.setInputCloud (cloud);
    Eigen::Matrix3f eigvecs= pca.getEigenVectors();
    float det= pcl::determinant3x3Matrix(eigvecs);
    if(det < 0) eigvecs = -eigvecs; // Guarantee a rotation. sigh PCL!
    
    Eigen::Vector4f segmean;
    segmean = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cloud, segmean);
    int nn= cloud->points.size();
    PointCloud::Ptr proj(new PointCloud);
    Normals::Ptr nproj(new Normals);
    proj->points.resize(nn);
    nproj->points.resize(nn);
    for(int j=0;j<nn;j++) {
        // project points (after subbing the mean)
        Eigen::Vector3f di= cloud->points[j].getVector3fMap() - segmean.head<3>();
        proj->points[j].getVector3fMap() = eigvecs.transpose() * di;

        // project the normals
        nproj->points[j].getNormalVector3fMap() = eigvecs.transpose() * normals->points[j].getNormalVector3fMap();
    }
    proj->width= 1;
    proj->height= nn;
    nproj->width= 1;
    nproj->height= nn;
    
    Eigen::Vector3f eigvals= pca.getEigenValues();
    eigvals /= eigvals.maxCoeff();
    
    // outputs
    lambda0normalized= eigvals(0);
    lambda1normalized= eigvals(1);
    lambda2normalized= eigvals(2);
    projcloud= proj;
    projnormals= nproj;
}

// Return mean of distances from points in cloud1 to their nearest neighbors in cloud2 
// Also take into account of the angle between the normals of the two points 
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
float cloudAlignmentScoreDenseWithNormalsNormalized(const PointCloud::Ptr &cloud1, const Normals::Ptr &ncloud1,  const PointCloud::Ptr &cloud2, const Normals::Ptr &ncloud2, float relweight, float dnormalize) {
    
  vector<int> ix;
  vector<float> dd;
  TreeT nntree;
  nntree.setInputCloud(cloud2);
  int N= cloud1->points.size();
  float accum=0;
  for(int i=0;i<N;i++) {
    nntree.nearestKSearch(cloud1->points[i], 1, ix, dd);
    
    accum += sqrt(dd[0]) / dnormalize; // normalize by provided length (usually extent of object)
    // also compare normals
    float dot= ncloud1->points[i].getNormalVector3fMap().dot(ncloud2->points[ix[0]].getNormalVector3fMap());
    accum += relweight*(1.0 - dot);

  }
  return accum/N;
}

// Return mean of distances from points in cloud1 to their nearest neighbors in cloud2 
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
float cloudAlignmentScoreDense(const PointCloud::Ptr &cloud1, const PointCloud::Ptr &cloud2) {
  
  vector<int> ix;
  vector<float> dd;
  TreeT nntree;
  nntree.setInputCloud(cloud2);
  int N= cloud1->points.size();
  float accum=0;
  for(int i=0;i<N;i++) {
    nntree.nearestKSearch(cloud1->points[i], 1, ix, dd);
    accum += sqrt(dd[0]);
  }
  return accum/N;

}

// Subdivide the given mesh
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
MeshT::Ptr mesh_subdivide( const MeshT::ConstPtr &mesh, float thr) {
    
  // make a replicate for our new mesh
  MeshT::Ptr meshout(new MeshT);
  *meshout= *mesh;
  
  // unfold mesh, which is stored as ros message
  PointCloud::Ptr pcl_cloud (new PointCloud);
  pcl::fromPCLPointCloud2( mesh->cloud, *pcl_cloud );
  vector<pcl::Vertices> toadd;
  int processed= 0;
  for(size_t i=0;i<mesh->polygons.size();i++) {
    
    const pcl::Vertices& v= mesh->polygons[i];
    
    // Get the 3 points
    PointT& p1= pcl_cloud->points[v.vertices[0]];
    PointT& p2= pcl_cloud->points[v.vertices[1]];
    PointT& p3= pcl_cloud->points[v.vertices[2]];
    
    // Convert to eigen points
    Vector3d pe1(p1.x,p1.y,p1.z);
    Vector3d pe2(p2.x,p2.y,p2.z);
    Vector3d pe3(p3.x,p3.y,p3.z);
    
    // Find length of maximal edge
    float l1= (pe1-pe2).norm();
    float l2= (pe2-pe3).norm();
    float l3= (pe3-pe1).norm();
    float maxedge= max(max(l1, l2), l3);
    if(maxedge < thr) continue; // below threshold, let's not subdivide
    
    // Find edge points of the polygon
    Vector3d c1= (pe1+pe2)/2;
    Vector3d c2= (pe2+pe3)/2;
    Vector3d c3= (pe1+pe3)/2;
    
    //Vector3d vcenter= (pe1 + pe2 + pe3)/3.0;
    //Point_t pcenter( vcenter[0], vcenter[1], vcenter[2] );
    
    // Subdivide this polygon
    
    pcl::Vertices pnew;
    pnew.vertices.resize(3);
    
    int dontadd= -1;
    size_t n1,n2;
    // find indices of largest 2 edges
    if(l3<=l1 && l3<=l2) { 
      PointT pt1; pt1.x = c1[0]; pt1.y = c1[1]; pt1.z = c1[2];
      PointT pt2; pt2.x = c2[0]; pt2.y = c2[1]; pt2.z = c2[2];
#ifdef USE_COLOR
      pt1.rgb = blend_colors(p1.rgb, p2.rgb);
      pt2.rgb = blend_colors(p2.rgb, p3.rgb);
#endif
      pcl_cloud->points.push_back(pt1);  
      pcl_cloud->points.push_back(pt2);
      n1= pcl_cloud->points.size() - 2;
      n2= pcl_cloud->points.size() - 1;
      
      pnew.vertices[0]= v.vertices[0];
      pnew.vertices[1]= v.vertices[2];
      pnew.vertices[2]= n2;
      
      dontadd= 2;
    }
    else if(l2<=l1 && l2<=l3) { 
      PointT pt1; pt1.x = c3[0]; pt1.y = c3[1]; pt1.z = c3[2];
      PointT pt2; pt2.x = c1[0]; pt2.y = c1[1]; pt2.z = c1[2];
#ifdef USE_COLOR
      pt1.rgb = blend_colors(p1.rgb, p3.rgb);
      pt2.rgb = blend_colors(p1.rgb, p2.rgb);
#endif
      pcl_cloud->points.push_back(pt1);
      pcl_cloud->points.push_back(pt2);        
      n1= pcl_cloud->points.size() - 2;
      n2= pcl_cloud->points.size() - 1;
      
      pnew.vertices[0]= v.vertices[2];
      pnew.vertices[1]= v.vertices[1];
      pnew.vertices[2]= n2;
      
      dontadd= 1;
    }
    else{ //if(l1<=l2 && l1<=l3) { 
      PointT pt1; pt1.x = c2[0]; pt1.y = c2[1]; pt1.z = c2[2];
      PointT pt2; pt2.x = c3[0]; pt2.y = c3[1]; pt2.z = c3[2];
#ifdef USE_COLOR
      pt1.rgb = blend_colors(p2.rgb, p3.rgb);
      pt2.rgb = blend_colors(p1.rgb, p3.rgb);
#endif
      pcl_cloud->points.push_back(pt1);
      pcl_cloud->points.push_back(pt2);        
      n1= pcl_cloud->points.size() - 2;
      n2= pcl_cloud->points.size() - 1;
      
      pnew.vertices[0]= v.vertices[1];
      pnew.vertices[1]= v.vertices[0];
      pnew.vertices[2]= n2;
      
      dontadd= 0;
    }
        
    //overwrite one polygon
    meshout->polygons[i]= pnew;
    
    // add other polygons to our toadd vector
    for(size_t j=0;j<3;j++) {
      if(j == dontadd) continue;
      pnew.vertices[0]= v.vertices[j];
      pnew.vertices[1]= n1;
      pnew.vertices[2]= n2;
      toadd.push_back(pnew);
    }
    
    processed++;
  } 
  //printf("Subdivided total of %d/%d meshes.\n", processed, mesh->polygons.size());
  
  //add all new polygons in toadd to mesh->polygons
  //meshout->polygons.insert(meshout->polygons.end(), toadd.begin(), toadd.end());
  for(size_t k=0;k<toadd.size();k++) meshout->polygons.push_back(toadd[k]);
  
  //printf("Old #polygons = %d, New #polygons = %d.\n", mesh->polygons.size(), meshout->polygons.size());
  
  // Also copy over the new cloud of points to new mesh
  pcl_cloud->width= pcl_cloud->points.size();
  pcl_cloud->height= 1;
  pcl::toPCLPointCloud2( *pcl_cloud, meshout->cloud );
  return meshout;
}


// Recursively subdivide the given mesh
// This function is referenced from:
// *Source*: A. Karpathy, S. Miller, and L. Fei-Fei, “Object discovery in 3d scenes via shape analysis,”
// IEEE International Conference on Robotics and Automation, 2013.
MeshT::Ptr mesh_subdivide_recursive( const MeshT::ConstPtr &mesh, float thr) {
  MeshT::Ptr mesh2(new MeshT);
  *mesh2= *mesh;
  while(true) {
    int num_before= mesh2->polygons.size();
    mesh2= mesh_subdivide(mesh2, thr);
    int num_after= mesh2->polygons.size(); 
    if(num_after==num_before) return mesh2;
  }
}

#endif

