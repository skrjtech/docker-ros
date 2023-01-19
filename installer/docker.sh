#!/bin/bash
VERSION=$1
git clone https://github.com/skrjtech/linux-setup.git
. linux-setup/installer/docker.sh $VERSION
rm -rf linux-setup