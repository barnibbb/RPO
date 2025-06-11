[//]: # (Image References)

[image1]: ./assets/fig_system.png "system"
[image2]: ./assets/fig_1a.png "1a"
[image3]: ./assets/fig_1b.png "1b"

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

## Experimental results

Disinfection coverages with only 0.38\% difference provided by 95 evenly distributed light source locations with equal proportions of the overall radiation time (left) and by the plan optimized with the proposed method containing only 5 radiation positions (right). The exposure level is expressed by the Viridis color scale with brighter values corresponding to a higher radiant dose, while the lamp locations are denoted by red squares. As can be seen, the optimized plan, albeit being much simpler, can yield almost identical results.

| 95 positions | 5 positions |
|:-------:|:-------:|
| ![image2] | ![image3] |

## Notes

- Augmented models require regeneration when new features are assigned to cells.
- Parameter names might be updated later + description.
- Frequently used parameters might be set as independent variables.
- Gurobi optimization should be run outside the container. It requires academic license in the form of gurobi.lic file and gurobipy installed.
