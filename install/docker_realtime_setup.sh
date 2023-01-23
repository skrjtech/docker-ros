#!/bin/bash
# Update And Upgrade
# sudo apt update && sudo apt -y upgrade 
# sudo apt-get update && sudo apt-get -y upgrade
# sudo apt install -y curl
# Realtime Kernel Install
# bash install/realtime_kernel_install.sh
# Docker Install And Pull Image Tag
# bash install/docker_install_khirobot_setup.sh
# Shutdown 
# sudo shutdown now

# Seting RunTime US
# sudo mkdir -p /sys/fs/cgroup/cpu,cpuacct/docker
# echo 950000 | sudo tee /sys/fs/cgroup/cpu,cpuacct/docker/cpu.rt_runtime_us
# docker restart
# sudo systemctl restart docker 
# Add Realtime Group 
# sudo addgroup realtime
# sudo usermod -aG realtime $(whoami)
# echo "@realtime - rtprio 99"  | sudo tee -a /etc/security/limits.conf
# echo "@realtime - priority 99" | sudo tee -a /etc/security/limits.conf
# echo "@realtime - memlock 512000" | sudo tee -a/etc/security/limits.conf
# Add Alias
# echo 'alias khidocker="docker run --rm -it --name khi_robot --cpu-rt-runtime=950000 --ulimit rtprio=99 --cap-add=sys_nice skrjtech/khi_robot:noetic"' >> ~/.bashrc
# Reboot 
# sudo reboot
