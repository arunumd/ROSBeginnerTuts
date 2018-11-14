# ROSBeginnerTuts
This is the Week 11 exercise related to publishing and subscribing to tf frames; recording and playing back bag files and also testing ros code using rostest/gtest.

## Overview
This repository contains a publisher - subscriber communication interface in ROS. The publisher node registers itself with the ROS master (roscore) and then broadcasts a simple message on a topic called "chatter". The listener node subscribes to the broadcasted topic and prints out the received message on the screen. The publisher node also publishes tf messages of fixed parameters. A bag file gets invoked along with the publisher node and records all the published topics. Later the bag file can be used to broadcast messages to the listener node. A simple test case using rostest has also been implemented.

## Assumptions
This project assumes the following: 

 - the user has ROS Version Kinetic Installed on an Ubuntu desktop/ laptop
 - the user has CMake installed on the same desktop/ laptop
 - the user has C++ 11 compatible GCC compiler installed on the same desktop/ laptop
 - the user also has the tf and rostest libraries as part of their ROS distribution

## Standard install via command-line

**Cloning the repository and integrating in your workspace**

```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_make
cd src
git clone -b Week11_HW https://github.com/arunumd/ROSBeginnerTuts.git
cd..
catkin_make
```

**Preliminary**

In a new terminal
```
roscore
```

**IMPORTANT :- TO RUN ALL OF THE TESTS BELOW, PLEASE DO NOT CLOSE YOUR ROS MASTER(roscore)**

**To run the talker node**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker 10
```

**To run the listener node**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

**To call the service**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosservice call /modifyContents yourstring
```

Note:- The keyterm yourstring above means your custom string which you desire to publish.

**To run all nodes with rosbag recording 'ON' using launch file**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch rosbagFlag:=true
```

**To view the tf frames**

Approach 1:

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun tf tf_echo world talk
```

Approach 2:

In a new terminal
```
rosrun rqt_tf_tree rqt_tf_tree 
```
or
```
rqt &
```

**To generate pdf file of rqt tree**

In a new terminal
```
rosrun tf view_frames
```

**To run rostest/gtest**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
cd build
make run_tests
```

**Listener node listening only to the bag file**

In a new terminal
```
source /opt/ros/kinetic/setup.bash
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

In a new terminal
```
cd ~/.ros
rosbag play bagfile.bag
```

