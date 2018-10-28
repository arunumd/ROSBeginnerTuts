# ROSBeginnerTuts
This is my practice work done based on the beginner ROS tutorials found on ros wiki page

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
git clone https://github.com/arunumd/ROSBeginnerTuts
Copy-paste the cloned repository inside the /src folder of your catkin workspace from the beginner tutorials
```

**Sourcing the environment and running the nodes**

In a new terminal
```
roscore
```

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/your-catkin-workspace-from-beginner-turorials
source ./devel/setup.bash
catkin_make
rosrun beginner_tutorials talker
```

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/your-catkin-workspace-from-beginner-turorials
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
