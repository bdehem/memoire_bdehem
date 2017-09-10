## Launch files README

This folder contains launch files for the 'ucl_drone' package.

To use one of these files use the following command:
```
$ roslaunch ucl_drone <FILENAME>.launch <arg1>:=<value1> <arg2>:=<value2> ...
```
#### Files description

 * `fly.launch` Launch all nodes. Note: controller, pathplanning and strategy were not reimplemented since 3D slam was implemented
 * `benchmark.launch` Run SLAM algorithm with a rosbag file as video input
 * `make_benchmark.launch` Create a rosbag file to be used later with benchmark.launch
 * `only_computer_vision.launch` Run SLAM algorithm with drone's video as input, with a physical drone (not flying)


* folder `components` contains lower level launch files that are called by these launch files 
