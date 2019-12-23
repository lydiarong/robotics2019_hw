# Introduction
This repository can make a map from world file generated from Gazebo.
# Config Environment
```
sudo apt-get install ros-kinetic-pointcloud-to-laserscan
```
# Sourcing from multiple workspaces
```
cd ~/rmai_ws
source devel/setup.bash
cd ~/robotics2019_hw
catkin_make
source devel/setup.bash
```
# Run
```
roslaunch roslaunch task3_autonomy autonomy.launch
```
