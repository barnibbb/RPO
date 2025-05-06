[//]: # (Image References)

[image1]: ./assets/fig_segmented.png "segmented"
[image2]: ./assets/fig_exposure.png "exposure"
[image3]: ./assets/fig_system.png "system"

# RPO - Radiation Plan Optimization for UV Disinfection Robots

## Setup

```bash
./build_docker.sh
./run_docker.sh
cd /home/appuser/rpo
catkin init
catkin build
```

**Keywords**: 3D reconstruction, OctoMap representation, irradiance estimation, genetic algorithm based optimization, minimal set of required radiation positions

![image1]
Segmented 3D model of a test environment.
![image2]
High disinfection level achieved with only five radiation positions.

## Requirements

- Ubuntu 18.04
- ROS Melodic
- pcl_ros
- octomap_ros
- octomap_rviz_plugins (for rviz visualization)
- OpenMP (for parallel computing)

## Install

- Create "catkin_ws"
- Place RPO and octomap_rviz_plugins to "catkin_ws/src/"
- Run catkin build

## Run experiment

- Navigate to "RPO/experiments/test" folder
- Check "params.txt" for parameter check, set work folder to point to the test folder
- Run the following commands:

```console
roslaunch RPO rviz.launch
```

```console
rosrun RPO RPO
```

- Check short_reports and long_reports folders for results

## System overview

![image3]
