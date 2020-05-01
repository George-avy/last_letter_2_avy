#!/bin/bash

set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ros/catkin_ws/devel/setup.bash
# source ~/.bashrc
cd /home/ros/catkin_ws
# . /opt/ros/melodic/setup.bash && \
catkin_make
