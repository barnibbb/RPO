cmake_minimum_required(VERSION 3.16)
project(rpo LANGUAGES CXX CUDA)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)

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


# Parameter test
add_executable(Parameters
  test/parameter_test.cpp
  src/parameters.cpp
)

target_link_libraries(Parameters
  ${catkin_LIBRARIES}
  yaml-cpp
)


# Extension test
add_executable(Extension
  test/extension_test.cpp
  src/extended_octree.cpp
)

target_link_libraries(Extension
  ${catkin_LIBRARIES}
)


# CPU Digital Differential Analysis ray tracing
add_executable(CPU_DDA
  dev/src/dda_ray_tracing_cpu.cpp
)

target_link_libraries(CPU_DDA
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
)


# GPU Digital Differential Analysis ray tracing
add_executable(GPU_DDA
  dev/src/dda_ray_tracing_gpu.cu
)

target_link_libraries(GPU_DDA
  ${catkin_LIBRARIES}
)

set_target_properties(GPU_DDA PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)


# GPU irradiance calculation
add_executable(GPU_Irradiance
  src/compute_maps.cpp
  src/gpu_ray_tracing.cu
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/extended_octree.cpp
  src/parameters.cpp
)

target_link_libraries(GPU_Irradiance
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)

set_target_properties(GPU_Irradiance PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)


# Ceiling lamp
add_executable(CeilingLamp
  src/ceiling_lamp.cpp
  src/gpu_ray_tracing.cu
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/extended_octree.cpp
  src/parameters.cpp
)

target_link_libraries(CeilingLamp
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)

set_target_properties(CeilingLamp PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)



# Split model
add_executable(SplitModel
  dev/src/split_model.cpp
  src/extended_octree.cpp
  src/parameters.cpp
)

target_link_libraries(SplitModel
  ${catkin_LIBRARIES}
  yaml-cpp
)


# Compute coverage for known plan
add_executable(KnownPlan
  test/known_plan.cpp
  src/plan_generator.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/extended_octree.cpp
  src/parameters.cpp
)

target_link_libraries(KnownPlan
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)


# Run GA based optimization
add_executable(RPO
  src/main.cpp
  src/plan_generator.cpp
  src/ros_visualizer.cpp
  src/dose_calculator.cpp
  src/extended_octree.cpp
  src/parameters.cpp
)

target_link_libraries(RPO
  ${catkin_LIBRARIES}
  OpenMP::OpenMP_CXX
  yaml-cpp
)



add_library(extended_octree_lib STATIC
  src/extended_octree.cpp
  src/dose_calculator.cpp
)

set_target_properties(extended_octree_lib PROPERTIES 
  POSITION_INDEPENDENT_CODE ON
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

target_include_directories(extended_octree_lib PUBLIC include)

pybind11_add_module(extended_octree_module
  src/octree_bindings.cpp 
)

target_link_libraries(extended_octree_module PRIVATE
  extended_octree_lib
  ${catkin_LIBRARIES}
)





