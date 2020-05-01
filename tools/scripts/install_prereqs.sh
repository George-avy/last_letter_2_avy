#!/bin/bash

set -e

# Install required system packages
apt-get -y -q update
# apt-get -y install apt-utils && \
apt-get -y -q install sudo vim git wget

# Install gazebo and its dependencies
# Choose version depending on ROS version
apt-get -y -q install ros-${ROS_DISTRO}-gazebo-ros-pkgs

# Install the newer catkin_tools build system
apt-get -y -q install python-catkin-tools

# # Separate installation of Gazebo from repo
# # Unneeded, installed with gazebo-ros-pkgs
# case $ROS_DISTRO in
#     'kinetic')
#         echo '>>> Installing Gazebo 7'
#         apt-get install -y gazebo7
#         ;;
#     'melodic')
#         echo '>>> Installing Gazebo 9'
#         apt-get install -y gazebo9
#         ;;
#     *)
#         echo ">>> Unsupported ROS version: " $ROS_DISTRO
#         ;;
# esac
