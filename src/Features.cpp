#include "Features.h"

Features::~Features() {}

Features::Features(float res, octomap::ColorOcTreeNode::Color colorObj1, octomap::ColorOcTreeNode::Color colorObj2)
    : generator(rd()), distribution(0.5) {
  resolution = res;
  color_obj1 = colorObj1;
  color_obj2 = colorObj2;
}

EigenVector Features::computeFeatureDescriptor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_1,
                                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in_2) {
  int kAngleHistogramBins = 18;
  int kDistanceHistogramBins = 3;
  const EigenVector3D gravity(0, 0, -1);
  const Matrix3 rotation = Matrix3::Identity();

  Eigen::Vector4f xyz_centroid;
  compute3DCentroid(*cloud_in_1, xyz_centroid);
  const EigenVector3D centroid(xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]);

  // Feature computation starts here.
  std::vector<int> angle_1_histogram(kAngleHistogramBins, 0);
  std::vector<int> angle_2_histogram(kAngleHistogramBins, 0);
  const int totalVoxels = (cloud_in_1->width * cloud_in_1->height) * (cloud_in_2->width * cloud_in_2->height);
  EigenVector min_dist_vect = EigenVector::Constant(totalVoxels, 0.0);

  AlignedVector<EigenVector3D> centered_reference_points, reference_directions, pcl1_points;
  centered_reference_points.reserve(int(cloud_in_1->width * cloud_in_1->height));
  for (pcl::PointCloud<PointXYZ>::iterator it1 = cloud_in_1->begin(); it1 != cloud_in_1->end(); it1++) {
    EigenVector3D point1(it1->x, it1->y, it1->z);
    pcl1_points.push_back(point1);
    EigenVector3D reference_vector = point1 - centroid;
    centered_reference_points.push_back(reference_vector);
  }
  int x = 0;
  for (pcl::PointCloud<PointXYZ>::iterator it2 = cloud_in_2->begin(); it2 != cloud_in_2->end(); it2++) {
    for (size_t i = 0; i < centered_reference_points.size(); i++) {
      EigenVector3D query_point(it2->x, it2->y, it2->z);
      const EigenVector3D& reference_point = centered_reference_points[i];
      const EigenVector3D vector_1_to_2 = query_point - pcl1_points[i];
      const double distance = vector_1_to_2.norm();
      if (x < totalVoxels) {
        min_dist_vect[x] = distance;
        x++;
      }
      // Assumes gravity is normalized.
      const EigenVector3D normal_1 = reference_point.cross(gravity);  // reference direction should be point1-centroid
      const EigenVector3D normal_2 = vector_1_to_2.cross(reference_point);  // mixes direction should be point2-point1

      const double cos_angle_1 = reference_point.dot(vector_1_to_2) / (reference_point.norm() * vector_1_to_2.norm());
      const double cos_angle_2 = normal_1.dot(normal_2) / (normal_1.norm() * normal_2.norm());

      AddToAngleHistogram(cos_angle_1, &angle_1_histogram);
      AddToAngleHistogram(cos_angle_2, &angle_2_histogram);
    }
  }

  std::sort(min_dist_vect.data(), min_dist_vect.data() + min_dist_vect.size());
  const int percentageVoxels = int(totalVoxels * 0.01);
  int histArrayDist[3] = {0, 0, 0};
  for (int j = 0; j < percentageVoxels; j++) {
    if (min_dist_vect[j] <= 0.06) {
      histArrayDist[0]++;
    }
    if (min_dist_vect[j] > 0.06 && min_dist_vect[j] <= 0.20) {
      histArrayDist[1]++;
    }
    if (min_dist_vect[j] > 0.20) {
      histArrayDist[2]++;
    }
  }

  EigenVector angle_1_vector(kAngleHistogramBins);
  EigenVector angle_2_vector(kAngleHistogramBins);
  EigenVector distance_vector(kDistanceHistogramBins);

  for (int i = 0; i < kAngleHistogramBins; i++) {
    angle_1_vector[i] = angle_1_histogram[i];
    angle_2_vector[i] = angle_2_histogram[i];
  }
  for (int i = 0; i < kDistanceHistogramBins; i++) {
    distance_vector[i] = histArrayDist[i];
  }

  EigenVector feature_vector(2 * kAngleHistogramBins + kDistanceHistogramBins);

  const double sum_angle_1_vector = angle_1_vector.sum();
  angle_1_vector = angle_1_vector / sum_angle_1_vector;
  const double sum_angle_2_vector = angle_2_vector.sum();
  angle_2_vector = angle_2_vector / sum_angle_2_vector;
  const double sum_distance_vector = distance_vector.sum();
  distance_vector = distance_vector / sum_distance_vector;

  feature_vector << angle_1_vector, angle_2_vector, distance_vector;

  return feature_vector;
}

/** \brief Compute the Euclidean distance between two eigen vectors.
 * \param[in] p_src the first eigen vector
 * \param[in] p_tgt the second eigen vector
 */
double Features::euclidean_eigen(const EigenVector& p_src, const EigenVector& p_tgt) {
  return ((p_src - p_tgt).norm());
}

// 0 is perfect match and mismatch is infinite
double Features::chi2_dist(const EigenVector& p_src, const EigenVector& p_tgt) {
  // mask to avoid division by zero
  EigenVector masked_src(p_src.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector masked_tgt(p_tgt.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  double d = (((masked_src - masked_tgt).array().square()) / masked_src.array()).sum();
  return d;
}

double Features::chi2_dist_lmnn(const EigenVector& p_src, const EigenVector& p_tgt) {
  // mask to avoid division by zero
  EigenVector masked_src(p_src.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector masked_tgt(p_tgt.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  double d = 0.5 * (((masked_src - masked_tgt).array().square()) / (masked_src + masked_tgt).array()).sum();
  return d;
}

double Features::jensen_shannon_div(const EigenVector& p_src, const EigenVector& p_tgt) {
  EigenVector masked_src(p_src.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector masked_tgt(p_tgt.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector term_a =
      masked_src.segment<18>(0).array() *
      (2 * masked_src.segment<18>(0).array() / (masked_src.segment<18>(0).array() + masked_tgt.segment<18>(0).array()))
          .array()
          .log();
  EigenVector term_b =
      masked_tgt.segment<18>(0).array() *
      (2 * masked_tgt.segment<18>(0).array() / (masked_src.segment<18>(0).array() + masked_tgt.segment<18>(0).array()))
          .array()
          .log();
  double firstHist = 0.5 * (term_a + term_b).array().sum() / log(2);
  term_a =
      masked_src.segment<18>(18).array() * (2 * masked_src.segment<18>(18).array() /
                                            (masked_src.segment<18>(18).array() + masked_tgt.segment<18>(18).array()))
                                               .array()
                                               .log();
  term_b =
      masked_tgt.segment<18>(18).array() * (2 * masked_tgt.segment<18>(18).array() /
                                            (masked_src.segment<18>(18).array() + masked_tgt.segment<18>(18).array()))
                                               .array()
                                               .log();
  double secondHist = 0.5 * (term_a + term_b).array().sum() / log(2);
  term_a = masked_src.segment<3>(36).array() * (2 * masked_src.segment<3>(36).array() /
                                                (masked_src.segment<3>(36).array() + masked_tgt.segment<3>(36).array()))
                                                   .array()
                                                   .log();
  term_b = masked_tgt.segment<3>(36).array() * (2 * masked_tgt.segment<3>(36).array() /
                                                (masked_src.segment<3>(36).array() + masked_tgt.segment<3>(36).array()))
                                                   .array()
                                                   .log();
  double thirdHist = 0.5 * (term_a + term_b).array().sum() / log(2);
  return firstHist + secondHist + thirdHist;
}

double Features::kl_div(const EigenVector& p_src, const EigenVector& p_tgt) {
  EigenVector masked_src(p_src.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector masked_tgt(p_tgt.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  double firstHist = (masked_src.segment<18>(0).array() *
                      ((masked_src.segment<18>(0).array() / masked_tgt.segment<18>(0).array()).array().log()))
                         .sum();
  double secondHist = (masked_src.segment<18>(18).array() *
                       ((masked_src.segment<18>(18).array() / masked_tgt.segment<18>(18).array()).array().log()))
                          .sum();
  double thirdHist = (masked_src.segment<3>(36).array() *
                      ((masked_src.segment<3>(36).array() / masked_tgt.segment<3>(36).array()).array().log()))
                         .sum();

  return firstHist + secondHist + thirdHist;
}

double Features::bhattacharyya_dist(const EigenVector& p_src, const EigenVector& p_tgt) {
  EigenVector masked_src(p_src.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  EigenVector masked_tgt(p_tgt.unaryExpr([](float v) -> float { return v <= 0 ? 0.0000001 : v; }));
  double firstHist =
      -1 * log((masked_src.segment<18>(0).array() * masked_tgt.segment<18>(0).array()).array().sqrt().sum());
  double secondHist =
      -1 * log((masked_src.segment<18>(18).array() * masked_tgt.segment<18>(18).array()).array().sqrt().sum());
  double thirdHist =
      -1 * log((masked_src.segment<3>(36).array() * masked_tgt.segment<3>(36).array()).array().sqrt().sum());
  return firstHist + secondHist + thirdHist;
}

void Features::downsamplePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, float resolution) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(resolution, resolution, resolution);
  sor.filter(*cloud_out);

  std::cout << "PointCloud after filtering: " << cloud_out->width * cloud_out->height << " data points ("
            << pcl::getFieldsList(*cloud_out) << ")." << std::endl;
}

void Features::add2PItoNegAngle(float& angleRad) {
  if (angleRad < 0) {
    angleRad = angleRad + 2 * M_PI;
  }
}

float Features::convertToRadians(double d) {
  float radians = (d / 360) * (2.0 * M_PI);
  return radians;
}
