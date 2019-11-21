cmake_minimum_required(VERSION 2.8)

project(learning_spatial_relations)


SET(CMAKE_BUILD_TYPE Release)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
#Google-glog
find_package(glog 0.3.3 REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(PCL  REQUIRED  )
include_directories(${PCL_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(Eigen3 REQUIRED)
find_package(OpenMP)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11 -O3")
include_directories(EIGEN3_INCLUDE_DIR)

add_executable(imitateRelationFast src/imitateRelationFast.cpp src/VoxelObjects.cpp  src/Features.cpp)
target_link_libraries(imitateRelationFast ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} ${PYTHON_LIBRARIES})


