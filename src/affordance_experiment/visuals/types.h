#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> Normals;
typedef pcl::search::KdTree<PointT> TreeT;
typedef pcl::PolygonMesh MeshT;