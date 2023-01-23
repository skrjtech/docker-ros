#!/bin/bash
# Update And Upgrade
sudo apt update && sudo apt -y upgrade 
sudo apt-get update && sudo apt-get -y upgrade
sudo apt install -y curl
# Realtime Kernel Install
bash install/realtime_kernel_install.sh
# Docker Install And Pull Image Tag
bash install/docker_install_khirobot_setup.sh
# Shutdown 
sudo shutdown now
