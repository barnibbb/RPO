[//]: # (Image References)

[image1]: ./assets/fig_system.png "system"

# RPO - Radiation Plan Optimization for UV Disinfection Robots

## Setup

```bash
./build_docker.sh
./run_docker.sh
cd /home/appuser/rpo
catkin init
catkin build
source devel/setup.bash
```

## Run experiment

Creating augmented octomap format from color octomap:

```bash
rosrun rpo Augmentation <color_model> <surface> <visualize>
```

The mounted data folder should look like this:

```text
data/
├── models/
│   ├── *_augmented.ot
│   ├── *_color.ot
│   └── lamp_model.csv
└── params.yaml
```

Experiments can be run by specifying the params.yaml file in the data folder.

```bash
rosrun rpo RPO <param_file>
```

The results can be visualized in rviz by running it in parallel with the experiments.

```bash
roslaunch rpo rviz.launch
```

## System overview

These codes are related to our paper **Optimal UV Disinfection Plan Generation with Minimum Number of Radiation Positions** submitted to Robotics and Autononomus Systems.

![image1]

## Notes

- Augmented models require regeneration when new features are assigned to cells.
- Parameter names might be updated later + description.
- Frequently used parameters might be set as independent variables.
