#!/bin/bash

set -e

# Create user
useradd -ms /bin/bash ros
adduser ros sudo
adduser root sudo
echo "%sudo   ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers

# RUN adduser --gecos "ROS User" --disabled-password ros
# RUN usermod -a -G dialout ros
