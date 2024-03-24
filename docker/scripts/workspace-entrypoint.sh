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
echo "source install/setup.bash" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc


source /opt/ros/${ROS_DISTRO}/setup.bash
source install/local_setup.bash  

# sudo apt-get update
# rosdep update
export ROS_LOCALHOST_ONLY=1

# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v4

# export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}

# export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

export PATH="$HOME/.local/bin:$PATH"

sudo usb_resetter --reset-all
# Restart udev daemon
sudo service udev restart

colcon build --symlink-install

sudo usb_resetter --reset-all

$@
