# Metric Learning for Generalizing Spatial Relations to New Objects
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/mees/generalize_spatial_relations.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/mees/generalize_spatial_relations/context:cpp)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/mees/generalize_spatial_relations.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/mees/generalize_spatial_relations/alerts/)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository implements the code for describing and generalizing spatial relations of our paper published at IROS 2017. More information at our [project page](http://spatialrelations.cs.uni-freiburg.de).

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

## Dataset
The Freiburg Spatial Relations [dataset](http://spatialrelations.cs.uni-freiburg.de/#dataset) features 546 scenes each containing two out of 25 household objects. The depicted spatial relations can roughly be described as on top, on top on the corner, inside, inside and inclined, next to, and inclined.
The dataset contains the 25 object models as textured .obj and .dae files, a low resolution .dae version for visualization in rviz, a scene description file containing the translation and rotation of the objects for each scene, a file with labels for each scene, the 15 splits used for cross validation, and a bash script to convert the models to pointclouds.

<p align="center">
  <img src="http://spatialrelations.cs.uni-freiburg.de/images/scene_examples.png" width="75%"/>
</p>

## Installation
Install [Glog](https://github.com/google/glog), [PCL](http://pointclouds.org/), [Eigen3](http://eigen.tuxfamily.org/) and [OctoMap](https://github.com/OctoMap/octomap). 
- Download the [dataset](http://spatialrelations.cs.uni-freiburg.de/#dataset) and extract it such that the structure looks like ```generalize_spatial_relations/data/object_models/```
- Navigate to the relations dataset and create the point clouds from the .obj files (needs pcl):
  ```cd scripts; ./create_uniform_pcd.sh```
- Put your directories path [here](https://github.com/mees/generalize_spatial_relations/blob/master/src/imitateRelationFast.cpp#L96). 

- Compile the c++ code: 
   ```
   mkdir build
   cd build
   cmake ..
   make -j4
   ```

## Experiments
To generalize relation from one or multiple scenes to another, take a look at ```imitateRelationsFast.cpp```. It uses a precomputed LMNN embedding to compute distances between spatial relations. For example to generalize the scenes _spearmint_salz_2662_ and _pot4_milkCarton_6394_ with a box object as base and a cereal box as the relative object:

```./imitateRelationFast -b SmallBoxBase -o muesli2 -i 8868  -1 spearmint_salz_2662 -2 pot4_milkCarton_6394```

If you are just interested in computing our spatial relation feature descriptor given two object clouds, take a look [here](https://github.com/mees/generalize_spatial_relations/blob/master/src/Features.cpp#L17).

## Notes
If you want to learn the features describing pairwise object relations with deep learning and optimize the generalization experiments on a continuous level check out the repo of our follow-up work [here](https://github.com/PhilJd/generalize_spatial_relations).

## License
For academic usage, the code is released under the [GPLv3](https://www.gnu.org/licenses/gpl-3.0.en.html) license. For any commercial purpose, please contact the authors. 