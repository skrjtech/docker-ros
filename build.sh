#!/bin/bash
# Update And Upgrade
sudo apt update && sudo apt -y upgrade 
sudo apt-get update && sudo apt-get -y upgrade
# Install Docker 
. installer/docker.sh 5:20.10.21~3-0~ubuntu-$(lsb_release -sc)
# Docker Pull KHI_ROBOT Image
docker pull skrjtech/khi_robot:melodic
docker pull skrjtech/khi_robot:noetic
# Install Realtime
. installer/realtime.sh
# Seting RunTime US
sudo echo 950000 > /sys/fs/cgroup/cpu/docker/cpu.rt_runtime_us