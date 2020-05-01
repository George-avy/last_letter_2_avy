#!/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ../ && catkin_make -j4
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /home/ros/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "echo ~/.bashrc loaded" >> ~/.bashrc