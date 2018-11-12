# ROSBeginnerTuts
This is the Week 10 exercise about adding ROS Log messages, creating a service and finding logs on rqt graph.

## Overview
This repository contains a publisher - subscriber communication interface in ROS. The publisher node registers itself with the ROS master (roscore) and then broadcasts a simple message on a topic called "chatter". The listener node subscribes to the broadcasted topic and prints out the received message on the screen.

## Assumptions
This project assumes the following: 
 - the user has gone through the beginner tutorials (1. Core ROS Turorials --> 1.1 Beginner Level (from 1 to 13)) available on [ROS Wiki Page](http://wiki.ros.org/ROS/Tutorials)
 - the user has ROS Version Kinetic Installed on an Ubuntu desktop/ laptop
 - the user has CMake installed on the same desktop/ laptop
 - the user has C++ 11 compatible GCC compiler installed on the same desktop/ laptop

## Standard install via command-line

**Cloning the repository and integrating in your workspace**

```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
cd src
git clone -b Week10_HW https://github.com/arunumd/ROSBeginnerTuts.git
git checkout Week10_HW
cd..
catkin_make
```

**Sourcing the environment and running the nodes**

In a new terminal
```
roscore
```

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker 10
```

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
