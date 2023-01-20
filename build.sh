#!/bin/bash
# Update And Upgrade
sudo apt update && sudo apt -y upgrade 
sudo apt-get update && sudo apt-get -y upgrade
sudo apt install -y curl
# Install Docker 
bash <(curl https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/docker.sh) 5:20.10.21~3-0~ubuntu-$(lsb_release -sc)
# Docker Pull KHI_ROBOT Image
sudo docker pull skrjtech/khi_robot:melodic
sudo docker pull skrjtech/khi_robot:noetic
# Install Realtime
. installer/realtime.sh
# Seting RunTime US
sudo mkdir -p /sys/fs/cgroup/cpu,cpuacct/docker
echo 950000 | sudo tee /sys/fs/cgroup/cpu,cpuacct/docker/cpu.rt_runtime_us
# docker restart
sudo systemctl restart docker 
# Add Realtime Group 
sudo addgroup realtime
sudo usermod -aG realtime $(whoami)
echo "@realtime - rtprio 99"  | sudo tee -a /etc/security/limits.conf
echo "@realtime - priority 99" | sudo tee -a /etc/security/limits.conf
echo "@realtime - memlock 512000" | sudo tee -a/etc/security/limits.conf
# Add Alias
echo 'alias khidocker="docker run --rm -it --name khi_robot --cpu-rt-runtime=950000 --ulimit rtprio=99 --cap-add=sys_nice skrjtech/khi_robot:noetic"' >> ~/.bashrc
# Reboot 
sudo reboot
