ARG BASE_IMG=ros:melodic-ros-core-bionic
FROM ${BASE_IMG}

# Use noninteractive debconf frontend since we're installing in a non-interactive way
ENV DEBIAN_FRONTEND noninteractive

# Select default shell
SHELL [ "/bin/bash", "-c" ]

RUN echo ">>> Building ROS ${ROS_DISTRO} for Ubuntu $(lsb_release -cs)"

# Copy over installation scripts
COPY ./tools/scripts /scripts

# Install required system packages
RUN . /scripts/install_prereqs.sh

# Create user
RUN . /scripts/create_user.sh

# Switch to created user
USER ros

# Create a new ROS workspace
RUN . /scripts/create_workspace.sh
# Copy the package under integration locally
COPY ./ /home/ros/catkin_ws/src/last_letter_2/

# Install package dependencies as root
RUN . /scripts/install_deps.sh

# Build packages as ros user
RUN . /scripts/build_source.sh

# RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Compile and test (mark the build as failed if any step fails). If the
# CATKIN_OPTIONS file exists, use it as an argument to catkin_make, for example
# to blacklist certain packages.
#
# NOTE on testing: `catkin_make run_tests` will show the output of the tests
# (gtest, nosetest, etc..) but always returns 0 (success) even if a test
# fails. Running `catkin_test_results` aggregates all the results and returns
# non-zero when a test fails (which notifies Travis the build failed).
# RUN source /opt/ros/$ROS_DISTRO/setup.bash
# RUN cd ~/catkin_ws
# RUN catkin_make

# ENTRYPOINT [ "/ros_entrypoint.sh" ]
# CMD ["bash"]

USER ros
# HOME needs to be set explicitly. Without it, the HOME environment variable is
# set to "/"
ENV HOME /home/ros
WORKDIR $HOME

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD [ "bash" ]