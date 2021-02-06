#ifndef FEATURES_H_
#define FEATURES_H_

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>
#include <random>

using namespace Eigen;
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::search;

typedef Eigen::Matrix3Xd EigenMatrix3D;
typedef Eigen::VectorXf EigenVector;
typedef Eigen::Vector3d EigenVector3D;
typedef Eigen::Vector3i EigenVector3i;

typedef Eigen::VectorXd Vector;
typedef Eigen::Matrix3d Matrix3;
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

constexpr double kAngularResolution = 0.17453;  // 10 deg;
// constexpr double kAngularResolution = 0.31415926535; // 19 deg;
constexpr double kZeroNorm = 0.0000001;

class Features {
 public:
  float resolution;
  octomap::ColorOcTreeNode::Color color_obj1;
  octomap::ColorOcTreeNode::Color color_obj2;
  std::random_device rd;
  std::mt19937 generator;
  std::bernoulli_distribution distribution;
  Features(float resolution, octomap::ColorOcTreeNode::Color color_obj1, octomap::ColorOcTreeNode::Color color_obj2);
  virtual ~Features();
  void add2PItoNegAngle(float &angleDeg);
  void downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float resolution);
  float convertToRadians(double d);

  double euclidean_eigen(const EigenVector &p_src, const EigenVector &p_tgt);
  double chi2_dist(const EigenVector &p_src, const EigenVector &p_tgt);
  double chi2_dist_lmnn(const EigenVector &p_src, const EigenVector &p_tgt);
  double bhattacharyya_dist(const EigenVector &p_src, const EigenVector &p_tgt);
  double kl_div(const EigenVector &p_src, const EigenVector &p_tgt);
  double jensen_shannon_div(const EigenVector &p_src, const EigenVector &p_tgt);
  EigenVector computeFeatureDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_1,
                                       const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_2);
  inline void AddToHistogram(double value, double bin_size, std::vector<int> *histogram) {
    const int index = value / bin_size;
    histogram->at(std::max(0, std::min(index, int(histogram->size()) - 1)))++;
  }
  // Hardcoded 18 bins, can be generalized.
  inline void AddToAngleHistogram(double cos_angle, std::vector<int> *histogram) {
    if (cos_angle < 0. /* cos(9/18 pi) */) {
      // angle > 9/18 pi
      if (cos_angle < -0.76604444311 /* cos(14/18 pi) */) {
        // angle > 14/18 pi
        if (cos_angle < -0.93969262078 /* cos(16/18 pi) */) {
          // angle > 16/18 pi
          if (cos_angle < -0.98480775301 /* cos(17/18 pi) */) {
            // angle > 17/18 pi
            histogram->at(17)++;
          } else {
            // angle <=17/18 and angle >16/18
            histogram->at(16)++;
          }
        } else {
          // angle <16/18 and angle > 14/18
          if (cos_angle < -0.86602540378 /* cos(15/18 pi) */) {
            // angle > 15/18 and angle <16/18
            histogram->at(15)++;
          } else {
            // angle<=15/18 and angle>14/18
            histogram->at(14)++;
          }
        }
      } else {
        // angle <= 14/18 and angle > 9/18
        if (cos_angle < -0.5 /* cos(12/18 pi) */) {
          // angle> 12/18 and angle<=14/18
          if (cos_angle < -0.64278760968 /* cos(13/18 pi) */) {
            // angle > 13/18 and angle<=14/18
            histogram->at(13)++;
          } else {
            // angle <= 13/18 and angle>12/18
            histogram->at(12)++;
          }
        } else {
          // angle <= 12/18 and angle > 9/18
          if (cos_angle < -0.34202014332 /* cos(11/18 pi) */) {
            // angle > 11/18 and angle <=12/18
            histogram->at(11)++;
          } else {
            // angle <=11/18 and angle>9/18
            if (cos_angle < -0.17364817766 /* cos(10/18 pi) */) {
              // angle > 10/18 and angle <=11/18
              histogram->at(10)++;
            } else {
              // angle <=10/18 and angle > 9/18
              histogram->at(9)++;
            }
          }
        }
      }
    } else {
      // angle <= 9/18 pi
      if (cos_angle < 0.64278760968 /* cos(5/18 pi) */) {
        // angle > 5/18 and angle <= 9/18 pi
        if (cos_angle < 0.34202014332 /* cos(7/18 pi) */) {
          // angle > 7/18 and angle <= 9/18 pi
          if (cos_angle < 0.17364817766 /* cos(8/18 pi) */) {
            // angle > 8/18 and angle <= 9/18 pi
            histogram->at(8)++;
          } else {
            // angle <=8/18 and angle > 7/18
            histogram->at(7)++;
          }
        } else {
          // angle <= 7/18 and angle > 5/18
          if (cos_angle < 0.5 /* cos(6/18 pi) */) {
            // angle > 6/18 and angle <=7/18
            histogram->at(6)++;
          } else {
            // angle <= 6/18 and angle >5/18
            histogram->at(5)++;
          }
        }
      } else {
        // angle<=5/18
        if (cos_angle < 0.86602540378 /* cos(3/18 pi) */) {
          // angle > 3/18 and angle <= 5/18
          if (cos_angle < 0.76604444311 /* cos(4/18 pi) */) {
            // angle > 4/18 and angle <=5/18
            histogram->at(4)++;
          } else {
            // angle <=4/18 and angle >3/18
            histogram->at(3)++;
          }
        } else {
          // angle <= 3/18
          if (cos_angle < 0.93969262078 /* cos(2/18 pi) */) {
            // angle >2/18 and angle <=3/18
            histogram->at(2)++;
          } else {
            // angle <=2/18
            if (cos_angle < 0.98480775301 /* cos(1/18 pi) */) {
              // angle > 1/18 and angle <=2/18
              histogram->at(1)++;
            } else {
              // angle <=1/18
              histogram->at(0)++;
            }
          }
        }
      }
    }
  }
};

#endif