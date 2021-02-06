#ifndef UTILS_H_
#define UTILS_H_

#include <boost/regex.hpp>

#include "VoxelObjects.h"

class Utils {
 public:
  static double RMSE(const Eigen::Vector3d &gt_translation, Eigen::Vector3d &estimated_translation) {
    return sqrt(((estimated_translation - gt_translation).array().square()).sum() / 3);
  }

  static double computeRMSE(const Eigen::Affine3f &gt_transform, Eigen::Affine3f &estimated) {
    Eigen::Vector3d trans_gt(gt_transform(0, 3), gt_transform(1, 3), gt_transform(2, 3));
    Eigen::Vector3d trans_est(estimated(0, 3), estimated(1, 3), estimated(2, 3));
    double rmse = RMSE(trans_gt, trans_est);
    return rmse;
  }

  static bool ComputeAngleBetweenTwoVectors2(const Eigen::Vector3f &vector1, const Eigen::Vector3f &vector2,
                                             double *angle) {
    assert(angle);
    double norm1 = vector1.norm();
    double norm2 = vector2.norm();
    if (norm1 < kZeroNorm || norm2 < kZeroNorm) {
      std::cout << "vector1 = " << vector1.transpose() << std::endl;
      std::cout << "vector2 = " << vector2.transpose() << std::endl;
      std::cerr << "Cannot compute angle between vectors if one is zero." << std::endl;
      return false;
    }

    const double cos_angle = vector1.dot(vector2) / (norm1 * norm2);

    *angle = acos(cos_angle);
    return true;
  }

  static bool ComputeAngleBetweenTwoVectors(const EigenVector3D &vector1, const EigenVector3D &vector2, double *angle) {
    assert(angle);
    double norm1 = vector1.norm();
    double norm2 = vector2.norm();
    if (norm1 < kZeroNorm || norm2 < kZeroNorm) {
      std::cout << "vector1 = " << vector1.transpose() << std::endl;
      std::cout << "vector2 = " << vector2.transpose() << std::endl;
      std::cerr << "Cannot compute angle between vectors if one is zero." << std::endl;
      return false;
    }

    const double cos_angle = vector1.dot(vector2) / (norm1 * norm2);

    *angle = acos(cos_angle);
    return true;
  }
  static double computeQuatError(const Eigen::Quaternionf &gt_quat, const Eigen::Quaternionf &estimated_quat,
                                 double &angle_deg) {
    double quatDist = computeGeodesicDistance(gt_quat.matrix(), estimated_quat.matrix(), angle_deg);
    return quatDist;
  }

  Eigen::Quaternionf sampleUniformQuaternion() {
    // Eigen generates uniform random numbers in [-1, 1], we need them in [0, 1].
    EigenVector3D u = EigenVector3D::Random().array().abs().matrix();
    Quaternionf rand_quat;
    rand_quat.w() = std::sqrt(1 - u[0]) * std::sin(2 * M_PI * u[1]);
    rand_quat.x() = std::sqrt(1 - u[0]) * std::cos(2 * M_PI * u[1]);
    rand_quat.y() = std::sqrt(u[0]) * std::sin(2 * M_PI * u[2]);
    rand_quat.z() = std::sqrt(u[0]) * std::cos(2 * M_PI * u[2]);
    // if (rand_quat.w()<0){
    // 	rand_quat.w()*=-1;
    // }
    // cout<<"rand_quat "<< rand_quat.w() << " and vector " << rand_quat.vec() << std::endl;
    return rand_quat;
  }

  static float degToRad(double d) {
    float radians = (d / 360) * (2.0 * M_PI);
    return radians;
  }

  static float radToDeg(double d) {
    float degrees = d * (180 / M_PI);
    return degrees;
  }

  static void testComputePoseError(const Eigen::Affine3f &gt_transform) {
    Eigen::Affine3f estimated = Eigen::Affine3f::Identity();
    estimated.translation() << -0.3, -0.3, -0.1;
    Eigen::Vector3d estimated_trans(-0.3, -0.3, -0.1);
    double theta_z = 90;
    double theta_x = 45;
    double theta_y = 45;
    estimated.rotate(Eigen::AngleAxisf(degToRad(theta_z), Eigen::Vector3f::UnitZ()));
    estimated.rotate(Eigen::AngleAxisf(degToRad(theta_x), Eigen::Vector3f::UnitX()));
    estimated.rotate(Eigen::AngleAxisf(degToRad(theta_y), Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf quat_est(estimated.rotation());
    Eigen::Quaternionf quat_gt(gt_transform.rotation());
    double rel_rot_deg = 0;
    cout << computeQuatError(quat_gt, quat_est, rel_rot_deg);
  }
  static double quaternionDistance(const Eigen::Quaternionf &gt_quat, Eigen::Quaternionf &estimated_quat) {
    return 1 - fabs(gt_quat.dot(estimated_quat));
  }

  static double computeGeodesicDistance(const Eigen::Matrix3f &rotation1, const Eigen::Matrix3f &rotation2,
                                        double &angle_deg) {
    Eigen::Matrix3f relative_rot = rotation1.transpose() * rotation2;
    Eigen::AngleAxisf test;
    test.fromRotationMatrix(relative_rot);
    double angle_rad = test.angle();
    angle_deg = radToDeg(angle_rad);
    double scaled_angle = angle_rad / (M_PI * 2);
    return scaled_angle;
  }

  static bool computeLogRotationEigenMatrix(const Eigen::Matrix3f &rot_matrix, Eigen::Matrix3f &log_rotation) {
    double scaled_trace = (rot_matrix.trace() - 1.0) / 2.0;
    if (scaled_trace > 0.999) {
      scaled_trace = 0.999;
    }
    if (scaled_trace < -0.999) {
      scaled_trace = -0.999;
    }

    double theta = acos(scaled_trace);
    if (fabs(theta) < 0.017 /*~1 deg*/) {
      log_rotation = Eigen::MatrixXf::Constant(3, 3, 0.0);
      return true;
    }

    // Avoid singularities at +/- pi.
    if (fabs(theta - M_PI) < 0.017 * 5 || fabs(theta - -M_PI) < 0.017 * 5) {
      return false;
    }

    log_rotation = rot_matrix - rot_matrix.transpose();
    log_rotation = log_rotation * (theta / (2 * sin(theta)));

    return true;
  }
  static EigenVector strToEigen(string line) {
    EigenVector vect = EigenVector::Constant(44, 0.0);
    std::vector<string> strs;
    boost::split(strs, line, boost::is_any_of(","));
    char ch1 = '[';
    char ch2 = ']';
    for (int i = 0; i < strs.size(); i++) {
      strs[i].erase(std::remove(strs[i].begin(), strs[i].end(), ch1), strs[i].end());
      strs[i].erase(std::remove(strs[i].begin(), strs[i].end(), ch2), strs[i].end());
      // cout<<strs[i]<<endl;
      vect[i] = atof(strs[i].c_str());
    }
    return vect;
  }

  static void transformBaseToGravity(pcl::PointCloud<pcl::PointXYZ>::Ptr &base_obj_cloud, string full_scene_name) {
    ifstream myfile_feat;
    myfile_feat.open("../simtrack_with_unit_gravity.txt");
    assert(myfile_feat.is_open());
    string line;
    std::vector<string> strs;
    std::vector<string> strs2;
    while (getline(myfile_feat, line)) {
      boost::split(strs, line, boost::is_any_of(":"));
      string filename = strs[0];
      if (full_scene_name == filename) {
        string transform_data = strs[1];
        boost::split(strs, transform_data, boost::is_any_of(";"));
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        if (strs[2].size() > 5 &&
            strs[2].size() < 100) {  // hack to avoid scenes where we didn't need to rotate the base due to gravity
          boost::split(strs2, strs[2], boost::is_any_of(","));

          //   //todo take rotation, w,x, y ,z quaternion
          Eigen::Quaternionf quat;
          quat.w() = atof(strs2[0].c_str());
          quat.x() = atof(strs2[1].c_str());
          quat.y() = atof(strs2[2].c_str());
          quat.z() = atof(strs2[3].c_str());
          transform.rotate(quat);
        }

        PointCloud<PointXYZ>::Ptr transformed_cloud(new PointCloud<PointXYZ>);
        pcl::transformPointCloud(*base_obj_cloud, *transformed_cloud, transform);
        pcl::copyPointCloud(*transformed_cloud, *base_obj_cloud);
      }
    }
  }

  static Vector3d get_GT_trans(string base_obj, string obj2, string id) {
    Vector3d gt_trans(0, 0, 0);
    string line;
    std::vector<string> strs;
    std::vector<string> strs2;
    ifstream myfile_feat;
    myfile_feat.open("../simtrack_with_unit_gravity.txt");
    assert(myfile_feat.is_open());
    string full_scene_name = base_obj + "_" + obj2 + "_" + id;
    while (getline(myfile_feat, line)) {
      boost::split(strs, line, boost::is_any_of(":"));
      string filename = strs[0];
      if (full_scene_name == filename) {
        string transform_data = strs[1];
        boost::split(strs, transform_data, boost::is_any_of(";"));
        boost::split(strs2, strs[0], boost::is_any_of(","));
        gt_trans(0) = atof(strs2[0].c_str());
        gt_trans(1) = atof(strs2[1].c_str());
        gt_trans(2) = atof(strs2[2].c_str());
      }
    }
    return gt_trans;
  }

  static Eigen::Quaternionf get_GT_quat(string base_obj, string obj2, string id) {
    Eigen::Quaternionf gt_quat;
    string line;
    std::vector<string> strs;
    std::vector<string> strs2;
    ifstream myfile_feat;
    myfile_feat.open("../simtrack_with_unit_gravity.txt");
    assert(myfile_feat.is_open());
    string full_scene_name = base_obj + "_" + obj2 + "_" + id;
    while (getline(myfile_feat, line)) {
      boost::split(strs, line, boost::is_any_of(":"));
      string filename = strs[0];
      if (full_scene_name == filename) {
        string transform_data = strs[1];
        boost::split(strs, transform_data, boost::is_any_of(";"));
        boost::split(strs2, strs[1], boost::is_any_of(","));
        gt_quat.w() = atof(strs2[0].c_str());
        gt_quat.x() = atof(strs2[1].c_str());
        gt_quat.y() = atof(strs2[2].c_str());
        gt_quat.z() = atof(strs2[3].c_str());
      }
    }
    return gt_quat;
  }

  static std::vector<EigenVector> computeTargetsFeatures(std::vector<string> targetsNames, string file_path) {
    std::vector<EigenVector> targets_features_vector;
    std::vector<string> strs;
    std::vector<string> strs2;
    string line;
    for (int i = 0; i < targetsNames.size(); i++) {
      ifstream myfile_feat;
      myfile_feat.open(file_path);
      assert(myfile_feat.is_open());
      while (getline(myfile_feat, line)) {
        boost::split(strs, line, boost::is_any_of("  <<"));
        string filename = strs[0];
        boost::split(strs, line, boost::is_any_of("<<"));  // strange hack needed
        string data = strs[2];
        if (filename == targetsNames[i] || filename == targetsNames[i] + "_rev") {
          EigenVector current_target = readLineIntoFeatVect(data);
          targets_features_vector.push_back(current_target);
        }
      }
    }
    return targets_features_vector;
  }

  static std::vector<EigenVector> computeTargetsFeatures(std::vector<string> targetsNames) {
    std::vector<EigenVector> targets_features_vector;
    std::vector<string> strs;
    std::vector<string> strs2;
    string line;
    for (int i = 0; i < targetsNames.size(); i++) {
      ifstream myfile_feat;
      myfile_feat.open("../features_all.txt");
      assert(myfile_feat.is_open());
      while (getline(myfile_feat, line)) {
        boost::split(strs, line, boost::is_any_of("  <<"));
        string filename = strs[0];
        boost::split(strs, line, boost::is_any_of("<<"));  // strange hack needed
        string data = strs[2];
        if (filename == targetsNames[i] || filename == targetsNames[i] + "_rev") {
          EigenVector current_target = readLineIntoFeatVect(data);

          targets_features_vector.push_back(current_target);
        }
      }
    }
    return targets_features_vector;
  }

  static EigenVector readLineIntoFeatVect(string line) {
    EigenVector target = EigenVector::Constant(39, 0.0);
    std::vector<string> strs;
    assert(target.size() == strs.size());
    boost::split(strs, line, boost::is_any_of(","));
    for (int i = 0; i < strs.size(); i++) {
      // cout<<strs[i]<<endl;
      target[i] = atof(strs[i].c_str());
    }

    // cout<<target.transpose()<<endl;
    return target;
  }

  template <typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
  static bool loadMatrix(std::string filename, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime> &m) {
    // General structure
    // 1. Read file contents into vector<double> and count number of lines
    // 2. Initialize matrix
    // 3. Put data in vector<double> into matrix
    std::ifstream input(filename.c_str());
    if (input.fail()) {
      std::cerr << "ERROR. Cannot find file '" << filename << "'." << std::endl;
      m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(0, 0);
      return false;
    }
    std::string line;
    Scalar d;
    std::vector<Scalar> v;
    std::vector<string> strs;
    int n_rows = 0;
    while (getline(input, line)) {
      ++n_rows;
      boost::split(strs, line, boost::is_any_of(","));
      for (int i = 0; i < strs.size(); i++) {
        d = atof(strs[i].c_str());
        v.push_back(d);
      }
    }
    input.close();
    int n_cols = v.size() / n_rows;
    m = Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>(n_rows, n_cols);

    for (int i = 0; i < n_rows; i++)
      for (int j = 0; j < n_cols; j++) m(i, j) = v[i * n_cols + j];

    return true;
  }

  int voxelize2(octomap::ColorOcTree *my_colorOctree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_a,
                const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_b, VoxelObjects *vo) {
    // vo->initializeOctomapWithFreeSpace(my_colorOctree);
    // fill colored OctTree
    pcl::PointCloud<PointXYZ>::iterator it;
    for (it = cloud_in_a->begin(); it != cloud_in_a->end(); ++it) {
      // Check if the point is invalid
      if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
        // cout<<"glob x: "<<glob_coordinates(0)<<"glob y: "<<glob_coordinates(1)<<"glob z: "<<glob_coordinates(2)<<"
        // voxel x: "<<vox_coordinates(0)<<" voxel y: "<<vox_coordinates(1)<<" voxel z: "<<vox_coordinates(2)<<endl;//"
        // array idx: "<<array_idx<<endl;
        my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
        my_colorOctree->setNodeColor(it->x, it->y, it->z, 20, 20, 255);
      }
    }
    bool intersects = false;
    for (it = cloud_in_b->begin(); it != cloud_in_b->end(); ++it) {
      // Check if the point is invalid
      if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
        // cout<<"x: "<<(*it).x<<" y: "<<(*it).y<<" z: "<<(*it).z<<endl;
        octomap::ColorOcTreeNode *result = my_colorOctree->search((*it).x, (*it).y, (*it).z);
        if (result != NULL) {  // if are not unknown
          octomap::ColorOcTreeNode::Color color = result->getColor();
          if (!my_colorOctree->isNodeOccupied(result) ||
              color != vo->color_obj1) {  // check it's free or it's the same object
            my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
            my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
          } else {
            // theta=theta-angle_increment;//try again
            // cout<<"Node occupied, the two objects intersect!"<<endl;

            intersects = true;
            // return -2;
          }
        } else {  // if area unknown
          // cout<<"why is it unknown??"<<endl;
          my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
          my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
          intersects = true;
        }
      }
    }

    if (intersects) return -2;
    return 0;
  }

  int voxelize(octomap::ColorOcTree *my_colorOctree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_a,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_b, VoxelObjects *vo) {
    vo->initializeOctomapWithFreeSpace(my_colorOctree);
    // fill colored OctTree
    pcl::PointCloud<PointXYZ>::iterator it;
    for (it = cloud_in_a->begin(); it != cloud_in_a->end(); ++it) {
      // Check if the point is invalid
      if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
        // cout<<"glob x: "<<glob_coordinates(0)<<"glob y: "<<glob_coordinates(1)<<"glob z: "<<glob_coordinates(2)<<"
        // voxel x: "<<vox_coordinates(0)<<" voxel y: "<<vox_coordinates(1)<<" voxel z: "<<vox_coordinates(2)<<endl;//"
        // array idx: "<<array_idx<<endl;
        my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
        my_colorOctree->setNodeColor(it->x, it->y, it->z, 20, 20, 255);
      }
    }
    bool intersects = false;
    for (it = cloud_in_b->begin(); it != cloud_in_b->end(); ++it) {
      // Check if the point is invalid
      if (!isnan(it->x) && !isnan(it->y) && !isnan(it->z)) {
        // cout<<"x: "<<(*it).x<<" y: "<<(*it).y<<" z: "<<(*it).z<<endl;
        octomap::ColorOcTreeNode *result = my_colorOctree->search((*it).x, (*it).y, (*it).z);
        if (result != NULL) {  // if are not unknown
          octomap::ColorOcTreeNode::Color color = result->getColor();
          if (!my_colorOctree->isNodeOccupied(result) ||
              color != vo->color_obj1) {  // check it's free or it's the same object
            my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
            my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
          } else {
            // theta=theta-angle_increment;//try again
            // cout<<"Node occupied, the two objects intersect!"<<endl;

            intersects = true;
            // return -2;
          }
        } else {  // if area unknown
          // cout<<"why is it unknown??"<<endl;
          my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
          my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
          intersects = true;
        }
      }
    }

    // my_colorOctree->write("bla.ot");
    if (intersects) return -2;
    return 0;
  }
};
#endif