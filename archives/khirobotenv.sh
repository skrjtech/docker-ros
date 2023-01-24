#!/bin/bash
source ~/.bashrc
sudo apt update
CATKIN_WS=/khi_robot
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