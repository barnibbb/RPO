cmake_minimum_required(VERSION 3.10.2)
project(rpo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap
  octomap_ros
  octomap_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Parameters
  test/parameter_test.cpp
  src/parameters.cpp
)

target_link_libraries(Parameters
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(Augmentation
  test/augmentation_test.cpp
  src/augmented_octree.cpp
)

target_link_libraries(Augmentation
  ${catkin_LIBRARIES}
)

add_executable(RPO
  src/main.cpp
  src/plan_generator.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(RPO
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)
