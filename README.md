# Local-Planner

+ A ROS package of a SAC-based-autonomous-navigation method
+ Use with repository [RL-RRT-Global-Planner](https://github.com/yiyunevin/Global-Planner.git)

## Environment

**System**
+ Ubuntu 20.04 
+ ROS Noetic (robot control, sensor data, communacate with global planner)

**Platform** 
+ KUKA youbot / Turtlebot3 Burger

**Anaconda Dependencies**
+ Python 3.6
+ pytorch 1.10.2
+ opencv-contrib-python 4.0.1.24
+ cudatoolkit 11.3.1
+ numpy
+ rospkg
+ yaml

## Usage

### Environment setting

1. Build a ROS package named `rrt_rl_nav`
2. Replace with the files in this repository
3. `$ catkin_make`

### Run the test program

**Simulation**

```bash
$ roslaunch rrt_rl_nav sim_set.launch
$ roslaunch rrt_planner global_planner.launch test:=false
## train
$ conda activate torch-gpu
$ python3 ~/rrt_rl_review_ws/src/rrt_rl_nav/script/main.py
## eval
$ conda activate torch-gpu
$ python3 ~/rrt_rl_review_ws/src/rrt_rl_nav/script/main.py --eval
```

**Real**

```bash
$ roslaunch rrt_rl_nav real_set.launch
$ roslaunch rrt_planner global_planner.launch test:=false
$ python3 ~/rrt_rl_review_ws/src/rrt_rl_nav/script/main.py --real
```
