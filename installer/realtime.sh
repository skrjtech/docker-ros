#!/bin/bash

Current=$PWD

rm -rf ~/kernel
mkdir -p ~/kernel
cd ~/kernel

wget https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-5.4.221.tar.gz
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.221-rt79.patch.gz

tar -xzf linux-5.4.221.tar.gz
gunzip patch-5.4.221-rt79.patch.gz

cd linux-5.4.221
patch -p1 < ../patch-5.4.221-rt79.patch

sudo apt install -y linux-image-5.4.0-54-generic
cp $Current/config/realtimeKernelConfig .config

sudo apt update && sudo apt -y upgrade
sudo apt-get update && sudo apt-get -y upgrade
sudo apt-get build-dep linux -y
sudo apt-get install -y libncurses-dev flex bison openssl libssl-dev dkms libelf-dev libudev-dev libpci-dev libiberty-dev autoconf fakeroot

yes '' | make oldconfig
# make menuconfig
make -j `nproc` deb-pkg
sudo dpkg -i ../*.deb

