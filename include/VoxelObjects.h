#ifndef VOXELOBJECTS_H_
#define VOXELOBJECTS_H_

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <random>
//#include <Eigen/StdVector>

#include <glog/logging.h>

#include "Features.h"

using namespace Eigen;
using namespace std;
using namespace pcl;

class VoxelObjects {
 private:
  float width, height, depth, resolution;
  boost::filesystem::path p1;
  boost::filesystem::path p2;
  octomap::ColorOcTree *my_colorOctree;
  Eigen::Affine3f current_transform;
  Eigen::Affine3f transform_noDataAugmentation;
  Features *feat;
  std::random_device rd;
  std::mt19937 generator;
  std::bernoulli_distribution distribution;

 public:
  VoxelObjects(float width, float height, float depth, float resolution, string filename1, string filename2);
  virtual ~VoxelObjects();
  octomap::ColorOcTreeNode::Color color_obj1;
  octomap::ColorOcTreeNode::Color color_obj2;
  void initializeOctomapWithFreeSpace(octomap::ColorOcTree *octree);
  bool voxelize(octomap::ColorOcTree *my_colorOctree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_a,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_b);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif