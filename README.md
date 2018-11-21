[![APMLicense](https://img.shields.io/apm/l/:package.svg)](https://raw.githubusercontent.com/AkshayRajaramanSubramanian/gazebot_walker/master/LICENSE.md)
# gazebot_walker
# ENPM 808X PROGRAMMING ASSIGNMENT
## Description
This package contains simple publisher and subscriber nodes, that allow a turtlebot simulation in gazebo to display simple obstacle avoidance behavior

## License
This project is licensed under the MIT License - see the [LICENSE](https://raw.githubusercontent.com/AkshayRajaramanSubramanian/gazebot_walker/master/LICENSE.md) file for details

## Dependencies
* ROS Kinetic
* Ubuntu 16.04
* catkin
* gazebo

## Package dependencies
cpp_common
rostime
roscpp_traits
roscpp_serialization
catkin
genmsg
genpy
message_runtime
std_msgs
geometry_msgs
gencpp
geneus
gennodejs
genlisp
message_generation
rosbuild
rosconsole
rosgraph_msgs
xmlrpcpp
roscpp
sensor_msgs

## Installing turtlebot_gazebo libraries
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
To install turtlebot for gazebo and its dependencies

## Using this package
Clone gazebot_walker repository
```
cd path_to_catkin_ws/src
git clone --recursive https://github.com/AkshayRajaramanSubramanian/gazebot_walker.git
```
Build package using catkin
```
cd path_to_catkin_ws
catkin_make
source devel/setup.bash
```
Running the launch file
```
roslaunch gazebot_walker gazebot_walker.launch
```
The launch file sets the record functionality to true by default,This can be toggled by setting the record parameter as follows:
```
roslaunch gazebot_walker gazebot_walker.launch record:='false'
```
The rosbag file will be saved in the 'bagfiles' directory

## Using the rosbag file
The recorded rosbag file is present within the 'bagfiles' directory. This can be played by running the commands:
```
rosbag play <filename>.bag
```

## Cpplint
Cpplint checks can be done by running the following commands:
```
cd path_to_catkin_ws/src/gazebot_walker
cpplint.py src/Walker.cpp src/main.cpp
```
The results from cpplint are stored in the 'results' directory


## Cppcheck
Cppcheck can be run by executing the following commands in the terminal:
```
cd path_to_catkin_ws/src/gazebot_walker
cppcheck --enable=all --check-config --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" -e "^./vendor/" )
```
The results from cppcheck are stored in the 'results' directory
