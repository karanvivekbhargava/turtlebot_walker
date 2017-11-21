<h1 align=center> Turtlebot Walker </h1>
<p align="center">
<a href='https://github.com/karanvivekbhargava/turtlebot_walker/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
</p>


## Project Overview

This project implements a roomba type behaviour on the commonly used turtlebot platform in ROS. It has a launch file `demo.launch` which launches the turtlebot gazebo simulation and also launches the custom `walker` node.

## Dependencies

This ROS node is made to be used on systems which have:
* ROS Kinetic
* Ubuntu 16.04
* Turtlebot packages

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

To install the turtlebot packages, run the following after installing ROS Kinetic on your ubuntu 16.04.

```
sudo apt-get install ros-kinetic-turtlebot-*
```

This installs all the turtlebot packages.

## How to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone --recursive https://github.com/karanvivekbhargava/turtlebot_walker.git
cd ..
catkin_make
```

## How to run demo

1. After following the installation instructions above, you can either run it using roslaunch by typing the following command in the terminal. It will start both the turtlebot gazebo simulation and the walker node in a separate terminal.
```
roslaunch turtlebot_walker demo.launch
```

2. If you'd like to run the nodes separately, then run roscore in the terminal as given below
```
roscore
```
We need to launch the turtlebot simulation. Run the command below in a new terminal.
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Then run the walker ros node by running the command below in a new terminal.
```
rosrun turtlebot_walker walker
```

## Running rostest

The unit tests have been written using gtest and rostest. To run the tests, you need to be in the catkin workspace parent folder. Then run the commands below

```
cd <path to catkin workspace>
catkin_make run_tests
```

You can test using

```
rostest turtlebot_walker walkerTest.launch
```

The output should be similar to the one below.

```
... logging to /home/karan/.ros/log/rostest-karan-ubuntu-10199.log
[ROSUNIT] Outputting test results to /home/karan/.ros/test_results/turtlebot_walker/rostest-test_walkerTest.xml
testwalkerTest ... ok

[ROSTEST]-----------------------------------------------------------------------

[turtlebot_walker.rosunit-walkerTest/collisionTest][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/karan/.ros/log/rostest-karan-ubuntu-10199.log
```

## Recording bag files with the launch file

You must first build the project using catkin_make as described earlier. You may run the command below to launch the nodes and record all the topics except the camera data. The bag file will be in the results directory once the recording is complete. By default it records for 30 seconds and you can change it in the optional argument `secs`

```
roslaunch turtlebot_walker demo.launch record:=true secs:=31
```

The bag file is stored in `..turtlebot_walker/results/turtlebot_walker.bag`

## Playing back the bag file

First, navigate to the results folder.

```
cd <path to repository>/results
```

To inspect the bag file, ensure that the roscore is running. Then in a new terminal, enter the command below while in the results directory.

```
rosbag play turtlebot_walker.bag
```

You will be able to see the elapsed time output on the screen. It would be playing the same messages that were recorded on all the recorded topics.

You can view all the messages being published on a topic e.g. `/cmd_vel_mux/input/navi`.

```
rostopic echo /cmd_vel_mux/input/navi
```
