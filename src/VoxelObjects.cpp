#include "VoxelObjects.h"

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", "");

VoxelObjects::VoxelObjects(float width, float height, float depth, float resolution, string filename1, string filename2)
    : width(width),
      height(height),
      depth(depth),
      resolution(resolution),
      p1(filename1),
      p2(filename2),
      generator(rd()),
      distribution(0.5) {
  my_colorOctree = new octomap::ColorOcTree(resolution);
  initializeOctomapWithFreeSpace(my_colorOctree);
  current_transform = Eigen::Affine3f::Identity();
  transform_noDataAugmentation = Eigen::Affine3f::Identity();
  color_obj1 = octomap::ColorOcTreeNode::Color(20, 20, 255);
  ;
  color_obj2 = octomap::ColorOcTreeNode::Color(230, 20, 20);
  ;
  feat = new Features(resolution, color_obj1, color_obj2);
}

VoxelObjects::~VoxelObjects() {
  delete feat;
}

void VoxelObjects::initializeOctomapWithFreeSpace(octomap::ColorOcTree *octree) {
  // int cont=0;
  for (float i = -(height / 2) + (resolution / 2); i <= height / 2; i = i + resolution) {
    for (float j = -(width / 2) + (resolution / 2); j <= width / 2; j = j + resolution) {
      for (float k = -(depth / 2) + (resolution / 2); k <= depth / 2; k = k + resolution) {
        octree->updateNode(i, j, k, false, false);
      }
    }
  }
}

bool VoxelObjects::voxelize(octomap::ColorOcTree *my_colorOctree, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_a,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in_b) {
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
      octomap::ColorOcTreeNode *result = my_colorOctree->search((*it).x, (*it).y, (*it).z);
      if (result != NULL) {  // if are not unknown
        octomap::ColorOcTreeNode::Color color = result->getColor();
        if (!my_colorOctree->isNodeOccupied(result) || color != color_obj1) {  // check it's free or it's the same
                                                                               // object
          my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
          my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
        } else {
          intersects = true;
          // return -2;
        }
      } else {  // if area unknown
        my_colorOctree->updateNode((*it).x, (*it).y, (*it).z, true, false);
        my_colorOctree->setNodeColor(it->x, it->y, it->z, 230, 20, 20);  // Red
        intersects = true;
      }
    }
  }

  return intersects;
}
