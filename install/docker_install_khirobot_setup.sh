#!/bin/bash
sudo apt update && sudo apt -y upgrade 
sudo apt-get update && sudo apt-get -y upgrade
sudo apt install -y curl

# Docker Install
bash <(curl https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/docker.sh) 5:20.10.21~3-0~ubuntu-$(lsb_release -sc)

#Docker Pull KHI_ROBOT Image
sudo docker pull skrjtech/khi_robot:melodic
sudo docker pull skrjtech/khi_robot:noetic