#!/bin/bash

source /opt/ros/noetic/setup.bash
sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update

sudo apt install -y cmake
sudo apt-get install -y python3-wstool python3-catkin-tools

CATKIN_WS=/catkin_ws
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS

catkin init
catkin build
cd $CATKIN_WS/src
git clone https://github.com/Kawasaki-Robotics/khi_robot.git
cd $CATKIN_WS
rosdep install -y -r --from-paths src --ignore-src
catkin build
echo "source ${CATKIN_WS}/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc