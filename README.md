# SEER: Safe Efficient Exploration with Learned Information


__Complete videos__: [video1](https://www.youtube.com/watch?v=5ZBkJmCKywg)


Please cite our paper if you use this project in your research:
- [__SEER: Safe Efficient Exploration for Aerial Robots using Learning to Predict Information Gain__](https://arxiv.org/abs/2209.11034), Yuezhan Tao, Yuwei Wu, Beiming Li, Fernando Cladera, Alex Zhou, Dinesh Thakur, Vijay Kumar.

```
@INPROCEEDINGS{10160295,
  author={Tao, Yuezhan and Wu, Yuwei and Li, Beiming and Cladera, Fernando and Zhou, Alex and Thakur, Dinesh and Kumar, Vijay},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={SEER: Safe Efficient Exploration for Aerial Robots using Learning to Predict Information Gain}, 
  year={2023},
  volume={},
  number={},
  pages={1235-1241},
  doi={10.1109/ICRA48891.2023.10160295}}
```

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.

## Table of Contents

- [SEER: Safe Efficient Exploration with Learned Information](#seer-safe-efficient-exploration-with-learned-information)
  - [Table of Contents](#table-of-contents)
  - [Quick Start](#quick-start)
  - [Exploring Different Environments](#exploring-different-environments)
  - [To Improve the Semantic Detection](#to-improve-the-semantic-detection)
    - [Disable Semantic Detection](#disable-semantic-detection)
  - [What is NOT Included in this Release](#what-is-not-included-in-this-release)
  - [Known issues](#known-issues)
    - [Compilation issue](#compilation-issue)
    - [libdw error](#libdw-error)

## Quick Start

This project has been tested on Ubuntu 20.04(ROS Noetic). Run the following commands to install required tools:

```
  sudo apt-get install libarmadillo-dev
  sudo apt-get install tmux
```

Install pytorch for running the occupancy predictor in simulation (verified with CUDA 11.7 and torch 1.12.0):
[torch install](https://pytorch.org/get-started/locally/)

Install catkin tools from official website:

[catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

After installing catkin tools, initialize workspace:

```
  cd ${YOUR_WORKSPACE_PATH}/src
  catkin init
  catkin config -DCMAKE_BUILD_TYPE=Release
```

Then simply clone and compile our package (using ssh here):

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone git@github.com:tyuezhan/SEER.git
  wstool init && wstool merge SEER/planner.rosinstall && wstool update
  cd ../
  catkin build
```

If you got any compilation issues, see [Known issues](#known-issues) section.

After compilation you can start a sample exploration demo. First, please add
```
source ${YOUR_WORKSPACE_PATH}/devel/setup.bash
```
to source this workspace in your ```.bashrc``` (.zshrc, etc. if you use other terminals). Since our tmux script will create new terminal windows, this is to make sure they can find the correct packages.

Firstly run ```roscd exploration_manager/scripts``` to navigate to the tmux script. 
Then, run
```
./tmux_sim.sh
```

This should launch tmux with multiple windows that contains the entire simulation stack.

By default you can see an office-like environment in Gazebo. The Rviz should show an empty environment with a small quadrotor. 

Firstly, click ```Motors On``` and ```Take Off``` from the mav manager GUI. 
Then, after you see some initial map, trigger the quadrotor to start exploration by the ```2D Nav Goal``` tool in ```Rviz```.

## Exploring Different Environments

The exploration environments in our simulator is in Gazebo. If you want to use your own environments, simply place the .world files in [planner/exploration_manager/worlds](planner/exploration_manager/worlds) and change the world argument in the tmux_gazebo_sim.launch file under [planner/exploration_manager/launch](planner/exploration_manager/launch).

You may also need to change the bounding box of explored space in tmux_exploration.launch under [planner/exploration_manager/launch](planner/exploration_manager/launch):

```xml
  <arg name="map_size_x" default="13"/>
  <arg name="map_size_y" default="15"/>
  <arg name="map_size_z" default="2.3"/>
  <arg name="box_min_x" default="-1.6"/>
  <arg name="box_min_y" default="-6.2"/>
  <arg name="box_min_z" default=" 0.0"/>
  <arg name="box_max_x" default="9.8"/>
  <arg name="box_max_y" default="7.0"/>
  <arg name="box_max_z" default=" 2.0"/>
  <arg name="map_origin_x" default="-1.5"/>
  <arg name="map_origin_y" default="-6.1"/>
  <arg name="map_origin_z" default="-0.1"/>
```

## To Improve the Semantic Detection
You may noticed that in this work we proposed a hand-crafted door detector. To improve the performance of detection, please feel free to replace the detection module with a NN model.
### Disable Semantic Detection
If you'd like to disable the semntic detection as well as the behavior state machine, flip the flag of semantic detection at line 133 in ```algorithm.xml``` in [planner/exploration_manager/launch](planner/exploration_manager/launch).

## What is NOT Included in this Release
1. Training data from MatterPort3D
2. Code for map reconstruction and training data generation 
3. Code for training the network

## Known issues

### Compilation issue

When running this project on Ubuntu 20.04, C++14 is required. Please add the following line in all CMakelists.txt files:

```
set(CMAKE_CXX_STANDARD 14)
```

### libdw error
```sudo apt-get install libdw-dev``` to solve compile issues


