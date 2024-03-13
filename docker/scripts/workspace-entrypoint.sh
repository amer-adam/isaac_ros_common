#!/bin/bash
#
# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

# Build ROS dependency
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /workspaces/isaac_ros-dev/install/setup.bash " >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_LOCALHOST_ONLY=1
source install/setup.bash 
# sudo apt-get update
# rosdep update

# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v4

# echo 1 | sudo update-alternatives --config cuda
# sudo rm -rf /usr/local/cuda-12.3 /usr/local/cuda-12 

export PATH=/usr/local/cuda-11.8/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# Restart udev daemon
sudo service udev restart

$@
