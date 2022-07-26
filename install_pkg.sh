#!/bin/bash

sudo apt install ros-${ROS_DISTRO}-ros-canopen -y
sudo apt install ros-${ROS_DISTRO}-can-msgs -y
sudo apt install ros-${ROS_DISTRO}-canopen-402 -y
sudo apt install ros-${ROS_DISTRO}-canopen-chain-node -y
sudo apt install ros-${ROS_DISTRO}-canopen-master -y
sudo apt install ros-${ROS_DISTRO}-canopen-motor-node -y
sudo apt install ros-${ROS_DISTRO}-socketcan-bridge -y
sudo apt install ros-${ROS_DISTRO}-socketcan-interface -y

sudo apt install ros-${ROS_DISTRO}-ros-control -y
sudo apt install ros-${ROS_DISTRO}-controller-interface -y
sudo apt install ros-${ROS_DISTRO}-controller-manager -y
sudo apt install ros-${ROS_DISTRO}-controller-manager-msgs -y
sudo apt install ros-${ROS_DISTRO}-hardware-interface -y
sudo apt install ros-${ROS_DISTRO}-joint-limits-interface -y
sudo apt install ros-${ROS_DISTRO}-transmission-interface -y
sudo apt install ros-${ROS_DISTRO}-dynamic-reconfigure -y
