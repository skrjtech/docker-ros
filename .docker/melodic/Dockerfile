FROM ubuntu:18.04

SHELL [ "/bin/bash", "-c" ]

RUN apt update && apt -y upgrade
RUN apt-get update && apt -y upgrade
RUN apt install -y sudo git curl wget make tmux x11-apps

# Python
RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/python.sh) 3.9.16
# Ros Melodic Install
RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/rosBeginner/main/installer/RosVersion1/melodic_version.sh)

COPY archives/khirobotenv.sh /khirobotenv.sh
COPY archives/khiRobotPack /khiRobotPack

CMD [ "/bin/bash"]