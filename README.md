# Metric Learning for Generalizing Spatial Relations to New Objects
This repository implements the code for describing and generalizing spatial relations of our paper published at IROS 2017.

## Reference
If you find the code helpful please consider citing our work 
```
@INPROCEEDINGS{mees17iros,
  author = {Oier Mees and Nichola Abdo and Mladen Mazuran and Wolfram Burgard},
  title = {Metric Learning for Generalizing Spatial Relations to New Objects},
  booktitle = {Proceedings of the International Conference on Intelligent Robots and Systems (IROS)},
  year = 2017,
  address = {Vancouver, Canada},
  url = {http://ais.informatik.uni-freiburg.de/publications/papers/mees17iros.pdf},
}
```

## Installation
Install [Glog](https://github.com/google/glog), [PCL](http://pointclouds.org/), [Eigen3](http://eigen.tuxfamily.org/) and [OctoMap](https://github.com/OctoMap/octomap). 
- Download the [dataset](http://spatialrelations.cs.uni-freiburg.de/#dataset) and extract it such that the structure looks like ```generalize_spatial_relations/data/object_models/```
- Navigate to the relations dataset and create the point clouds from the .obj files (needs pcl):
  ```cd scripts; ./create_uniform_pcd.sh```
- Put your directories path [here](https://github.com/mees/generalize_spatial_relations/blob/master/src/imitateRelationFast.cpp#L96). 

- Compile the c++ code: ```
mkdir build
cd build
cmake ..
make -j4
```