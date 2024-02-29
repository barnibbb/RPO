cmake_minimum_required(VERSION 3.10.2)
project(rpo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

set (octomap_DIR "/opt/ros/melodic/share/octomap/")

find_package(OpenMP REQUIRED)

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
)

add_executable(Augmentation
  test/augmentation_test.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
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
)

add_executable(EXP
  src/experiment.cpp
)

add_executable(ModelCleaner
  test/octomap_converter.cpp
)

target_link_libraries(ModelCleaner
  ${catkin_LIBRARIES}
)

add_executable(KnownPlan
  src/known_plan.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(KnownPlan
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(FloorPlan
  src/floor_plan.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(FloorPlan
  ${catkin_LIBRARIES}
)


add_executable(GZ
  src/ground_zone.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(GZ
  ${catkin_LIBRARIES}
)
