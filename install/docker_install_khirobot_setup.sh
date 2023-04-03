#!/bin/bash
sudo apt update && sudo apt -y upgrade 
sudo apt-get update && sudo apt-get -y upgrade
sudo apt install -y curl

# Docker Install
sudo apt install -y gnome-terminal
sudo apt remove docker-desktop
rm -r $HOME/.docker/desktop
sudo rm /usr/local/bin/com.docker.cli
sudo apt purge docker-desktop
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo chmod a+r /etc/apt/keyrings/docker.gpg
sudo apt-get update
VERSION='5:20.10.21~3-0~ubuntu-$(lsb_release -sc)'
sudo apt-get install -y docker-ce=$VERSION docker-ce-cli=$VERSION containerd.io docker-compose-plugin

#Docker Pull KHI_ROBOT Image
sudo docker pull skrjtech/khi-robot:melodic
sudo docker pull skrjtech/khi-robot:noetic
sudo usermod -aG docker ${USER}
su - ${USER}

