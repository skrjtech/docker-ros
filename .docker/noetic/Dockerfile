FROM ubuntu:20.04

SHELL [ "/bin/bash", "-c" ]

RUN apt update && apt -y upgrade
RUN apt-get update && apt -y upgrade
RUN apt install -y sudo git curl wget make tmux x11-apps

# Python
RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/python.sh) 3.9.16
# Ros Noetic Install
RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/rosBeginner/main/installer/RosVersion1/noetic_version.sh)
# KHIRobot Setting
RUN bash <(curl -sSL )

CMD [ "/bin/bash"]