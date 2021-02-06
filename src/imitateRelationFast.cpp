#include <getopt.h>
#include <omp.h>
#include <unistd.h>

#include <algorithm>

#include "Utils.h"
#include "VoxelObjects.h"

#define PSEUDOZERO 0.0000001

/*funcion that show the help information*/
void showhelpinfo(char *s) {
  cout << "Usage:   " << s << " [-option] [argument]" << endl;
  cout << "option:  "
       << "-b  base object of test scene" << endl;
  cout << "         "
       << "-o seconf object of the test scene" << endl;
  cout << "         "
       << "-i id of the test scene" << endl;
  cout << "         "
       << "-1  target scene 1 shown by user" << endl;
  cout << "         "
       << "-2  optional, target scene 2 shown by user" << endl;
  cout << "         "
       << "-3  optional, target scene 3 shown by user" << endl;
  cout << "         "
       << "-4  optional, target scene 4 shown by user" << endl;
  cout << "         "
       << "-5  optional, target scene 5 shown by user" << endl;
  cout << "example: " << s << " -b SmallBoxBase -o muesli2 -i 8868 -1 spearmint_salz_2662 -2 pot4_milkCarton_6394"
       << endl;
}

bool cmpPairs(const pair<float, int> &l, const pair<float, int> &r) { return l.first < r.first; }

int main(int argc, char *argv[]) {
  int opt;
  string base_obj;
  string obj2;
  string id;
  string L_file;
  string L_dense_file;
  string target1;
  string target2;
  string target3;
  string target4;
  string target5;
  if (argc == 1) {
    showhelpinfo(argv[0]);
    exit(1);
  }
  int lenOfTargets = 1;
  std::vector<string> targetsName;
  while ((opt = getopt(argc, argv, "b:o:i:1:2:3:4:5")) != -1) {
    switch (opt) {
      case 'b':  // base object of test scene
        base_obj = std::string(optarg);
        break;
      case 'o':  // second object
        obj2 = std::string(optarg);
        break;
      case 'i':  // scene id
        id = std::string(optarg);
        break;
      case '1':  // target 1 demonstrated by user
        target1 = std::string(optarg);
        targetsName.push_back(target1);
        break;
      case '2':  // target 1 demonstrated by user
        target2 = std::string(optarg);
        targetsName.push_back(target2);
        break;
      case '3':  // target 1 demonstrated by user
        target3 = std::string(optarg);
        targetsName.push_back(target3);
        break;
      case '4':  // target 1 demonstrated by user
        target4 = std::string(optarg);
        // cout<<"target4: "<<target4<<endl;
        targetsName.push_back(target4);
        break;
      case '5':  // target 1 demonstrated by user
        target5 = std::string(optarg);
        targetsName.push_back(target5);
        break;
      default:
        showhelpinfo(argv[0]);
        break;
    }
  }
  Utils utils;
  L_dense_file = "../L_dense.txt";
  string scene_dir = "YOUR_FOLDER_PATH";

  google::SetLogDestination(google::INFO, scene_dir.c_str());
  google::InitGoogleLogging(argv[0]);
  LOG(INFO) << "base_obj " << base_obj << endl;
  LOG(INFO) << "obj2 " << obj2 << endl;
  LOG(INFO) << "id " << id << endl;
  LOG(INFO) << "L_dense_file " << L_dense_file << endl;
  LOG(INFO) << "target1 " << target1 << endl;
  LOG(INFO) << "target2 " << target2 << endl;
  LOG(INFO) << "target3 " << target3 << endl;
  LOG(INFO) << "target4 " << target4 << endl;
  LOG(INFO) << "target5 " << target5 << endl;

  MatrixXf L;
  utils.loadMatrix(L_dense_file, L);

  float resolution = 0.01;
  Features *feat = new Features(resolution, octomap::ColorOcTreeNode::Color(20, 20, 255),
                                octomap::ColorOcTreeNode::Color(230, 20, 20));
  VoxelObjects *vo = new VoxelObjects(1.5, 1.5, 1.15, 0.01, base_obj, obj2);
  std::vector<EigenVector> targets_features_vector =
      utils.computeTargetsFeatures(targetsName, "../features_all_downsampled3cm.txt");
  std::vector<EigenVector> targets_features_vector_LMNN_dense;
  for (int i = 0; i < targets_features_vector.size(); i++) {
    targets_features_vector_LMNN_dense.push_back(L * targets_features_vector[i]);
  }
  Vector3d const gt_trans = utils.get_GT_trans(base_obj, obj2, id);
  Eigen::Quaternionf const gt_quat = utils.get_GT_quat(base_obj, obj2, id);
  PointCloud<PointXYZ>::Ptr cloud_in_3(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_in_4(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_3_downsampled(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_4_downsampled(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_3_downsampled1cm(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr cloud_4_downsampled1cm(new PointCloud<PointXYZ>);

  // // Read in the cloud data
  pcl::PCDReader reader;
  reader.read(("../data/object_models/" + base_obj + "/" + base_obj + ".pcd").c_str(), *cloud_in_3);
  reader.read(("../data/object_models/" + obj2 + "/" + obj2 + ".pcd").c_str(), *cloud_in_4);

  feat->downsamplePointCloud(cloud_in_3, cloud_3_downsampled1cm, 0.03f);  // 0.01f);
  feat->downsamplePointCloud(cloud_in_4, cloud_4_downsampled1cm, 0.03f);  // 0.01f);
  feat->downsamplePointCloud(cloud_in_3, cloud_3_downsampled, 0.03f);     // 0.01f);
  feat->downsamplePointCloud(cloud_in_4, cloud_4_downsampled, 0.03f);     // 0.01f);
  IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", "");

  bool consider_angles_to_gravity = true;
  double error = 10000;
  float thres = 0.001;
  int iter_num = 0;
  utils.transformBaseToGravity(cloud_3_downsampled, base_obj + "_" + obj2 + "_" + id);
  constexpr int numSampleRotations = 10;
  bool breakLoop = false;
  // set rand seed
  srand(1);
  constexpr float min_range = -0.2;
  constexpr float max_range = 0.2;
  constexpr float z_min_range = 0;
  constexpr float z_max_range = 0.2;
  constexpr float stepsize = 0.02;  // 0.01
  constexpr int coord_size = 7;
  constexpr int totalVoxels = pow(int((max_range - min_range) / stepsize) + 1, 2) * numSampleRotations * coord_size *
                              (((z_max_range - z_min_range) / stepsize) + 1);
  cout << "totalVoxels: " << totalVoxels << endl;
  double *poses_array = new double[totalVoxels];
  int array_index = 0;

  for (float x_trans = min_range; x_trans <= max_range; x_trans = x_trans + stepsize) {
    for (float y_trans = min_range; y_trans <= max_range; y_trans = y_trans + stepsize) {
      for (float z_trans = z_min_range; z_trans <= z_max_range; z_trans = z_trans + stepsize) {
        for (int idx = 0; idx < numSampleRotations; idx = idx + 1) {
          Eigen::Quaternionf current_quat;
          if (idx == 0) {
            current_quat.w() = 1;
            current_quat.x() = 0;
            current_quat.y() = 0;
            current_quat.z() = 0;
          } else if (idx == 1) {  // 90 degrees, to get lying shapes upright
            current_quat.w() = 0.707107;
            current_quat.x() = 0.707107;
            current_quat.y() = 0;
            current_quat.z() = 0;
          } else {
            current_quat = utils.sampleUniformQuaternion();
          }
          poses_array[array_index] = x_trans;
          poses_array[array_index + 1] = y_trans;
          poses_array[array_index + 2] = z_trans;
          poses_array[array_index + 3] = current_quat.w();
          poses_array[array_index + 4] = current_quat.x();
          poses_array[array_index + 5] = current_quat.y();
          poses_array[array_index + 6] = current_quat.z();
          array_index = array_index + coord_size;
        }
      }
    }
  }
  int np = omp_get_max_threads();
  auto start_feat = std::chrono::steady_clock::now();

  std::vector<std::pair<float, int> > concatenated_array;
  boost::shared_ptr<std::vector<std::pair<float, int> > > *global_array;
  global_array = new boost::shared_ptr<std::vector<std::pair<float, int> > >[np];
  int local_array_size = 15;

#pragma omp parallel
  {
    boost::shared_ptr<std::vector<std::pair<float, int> > > local_scores;  // score, id
    local_scores = boost::make_shared<std::vector<std::pair<float, int> > >();
    global_array[omp_get_thread_num()] = local_scores;
    // initialize local arrays with a high number
    for (int w = 0; w < local_array_size; w++) {
      local_scores->push_back(make_pair(1000, 1));
    }

#pragma omp for
    for (int idx = 0; idx < totalVoxels; idx = idx + coord_size) {
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.translation() << poses_array[idx], poses_array[idx + 1], poses_array[idx + 2];
      Eigen::Quaternionf current_quat;
      current_quat.w() = poses_array[idx + 3];
      current_quat.x() = poses_array[idx + 4];
      current_quat.y() = poses_array[idx + 5];
      current_quat.z() = poses_array[idx + 6];
      transform.rotate(current_quat);

      PointCloud<PointXYZ>::Ptr transformed_cloud(new PointCloud<PointXYZ>);
      transformed_cloud->clear();
      pcl::transformPointCloud(*cloud_4_downsampled, *transformed_cloud, transform);
      EigenVector feature_vector_current = feat->computeFeatureDescriptor(cloud_3_downsampled, transformed_cloud);

      EigenVector feature_vector_current_DenseLMNN_mapped = L * feature_vector_current;
      double miniLMNN_error = 0, denselmnn = 0;
      EigenVector errors_min = EigenVector::Constant(1, 100000.0);
      for (int i = 0; i < targets_features_vector.size(); i++) {
        denselmnn =
            feat->euclidean_eigen(targets_features_vector_LMNN_dense[i], feature_vector_current_DenseLMNN_mapped);

        if (denselmnn < errors_min[0]) {
          errors_min[0] = denselmnn;
        }
      }
      LOG(INFO) << "x_trans: " << poses_array[idx] << " y_trans: " << poses_array[idx + 1]
                << " z_trans: " << poses_array[idx + 2] << " quat: " << current_quat.w() << " " << current_quat.vec()[0]
                << " " << current_quat.vec()[1] << " " << current_quat.vec()[2] << " error_metric: " << errors_min[0]
                << endl;
      // check if we have a minima that is lower than the top 10
      if (errors_min[0] < local_scores->back().first) {
        std::pair<float, int> new_pair = make_pair(errors_min[0], idx);
        // check where we can insert the new minima in the sorted array
        int zenbakia =
            std::lower_bound(local_scores->begin(), local_scores->end(), new_pair, cmpPairs) - local_scores->begin();
        local_scores->insert(local_scores->begin() + zenbakia, new_pair);
        local_scores->pop_back();
      }
    }  // end for loop
  }    // end openmpi

  auto end_feat = std::chrono::steady_clock::now();
  cout << std::chrono::duration<double, std::milli>(end_feat - start_feat).count() << " ms features for loop"
       << std::endl;
  cout << std::chrono::duration<double, std::ratio<60> >(end_feat - start_feat).count() << " minutes features for loop"
       << std::endl;
  octomap::ColorOcTree *my_colorOctree2 = new octomap::ColorOcTree(resolution);
  vo->initializeOctomapWithFreeSpace(my_colorOctree2);

  for (int i = 0; i < np; i++) {
    concatenated_array.insert(concatenated_array.end(), global_array[i]->begin(), global_array[i]->end());
  }
  cout << "concatenated_array size " << concatenated_array.size() << endl;
  std::sort(concatenated_array.begin(), concatenated_array.end(), cmpPairs);
  for (int i = 0; i < concatenated_array.size(); i++) {
    octomap::ColorOcTree colorOctree_out(*my_colorOctree2);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    int idx = concatenated_array[i].second;  // id in pose array
    transform.translation() << poses_array[idx], poses_array[idx + 1], poses_array[idx + 2];
    Eigen::Quaternionf estimated_quat;
    estimated_quat.w() = poses_array[idx + 3];
    estimated_quat.x() = poses_array[idx + 4];
    estimated_quat.y() = poses_array[idx + 5];
    estimated_quat.z() = poses_array[idx + 6];
    transform.rotate(estimated_quat);
    PointCloud<PointXYZ>::Ptr transformed_cloud(new PointCloud<PointXYZ>);
    transformed_cloud->clear();
    pcl::transformPointCloud(*cloud_4_downsampled1cm, *transformed_cloud, transform);
    int success = utils.voxelize2(&colorOctree_out, cloud_3_downsampled1cm, transformed_cloud, vo);
    string intersects = "yes";
    if (success == 0) {  // does not intersect
      intersects = "no";
    }
    Eigen::Vector3d estimated_trans(poses_array[idx], poses_array[idx + 1], poses_array[idx + 2]);
    double rmse = utils.RMSE(gt_trans, estimated_trans);
    double rel_angle_deg = 0;
    //   double quat_error = utils.computeRotErrorWithSymmetries(obj2_symm, gt_quat, estimated_quat);
    double quat_error = utils.computeQuatError(gt_quat, estimated_quat, rel_angle_deg);
    //   double pose_error = rmse+quat_error;
    LOG(INFO) << "intersects: " << intersects << ", LMNN optimum at " << poses_array[idx] << ", "
              << poses_array[idx + 1] << ", " << poses_array[idx + 2] << ", quat: " << estimated_quat.w() << " "
              << estimated_quat.x() << " " << estimated_quat.y() << " " << estimated_quat.z()
              << " with min-policy error of " << concatenated_array[i].first << ", translation error: " << rmse
              << ", rot error " << quat_error << endl;
  }
  delete poses_array;
}
