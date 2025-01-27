cmake_minimum_required(VERSION 3.10.2)
project(rpo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

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


add_executable(RTT
  test/ray_tracing_test.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(RTT
  ${catkin_LIBRARIES}
)

add_executable(BST
  test/binary_save.cpp
  src/augmented_octree.cpp
  src/parameters.cpp
)

target_link_libraries(BST
  ${catkin_LIBRARIES}
)


add_executable(CM
  src/augmented_octree.cpp
  test/compare_models.cpp
)

target_link_libraries(CM
  ${catkin_LIBRARIES}
)

add_executable(RT3
  src/augmented_octree.cpp
  test/ray_tracing_3d.cpp
)

target_link_libraries(RT3
  ${catkin_LIBRARIES}
)


add_executable(PLY2OCTOMAP
  test/ply2octomap.cpp
  src/augmented_octree.cpp
)

target_link_libraries(PLY2OCTOMAP
  ${catkin_LIBRARIES}
)

add_executable(H2C
  test/height_color.cpp
)

target_link_libraries(H2C
  ${catkin_LIBRARIES}
)

