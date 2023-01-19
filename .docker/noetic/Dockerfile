FROM ubuntu:20.04
SHELL [ "/bin/bash", "-c" ]
RUN apt update && apt -y upgrade
RUN apt-get update && apt -y upgrade
RUN apt install -y sudo git curl wget make tmux x11-apps
# Install Python 3.7.16
RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/pycharm.sh) 3.7.16
ENV PATH /usr/local/python/bin:$PATH
# Ros Noetic Install
RUN git clone https://github.com/skrjtech/rosBeginner.git
RUN DEBIAN_FRONTEND=noninteractive . rosBeginner/build.sh
RUN rm -rf rosBeginner
COPY ../../installer/khirobotenv.sh /khirobotenv.sh
COPY ../../sources/khiRobotPack /khiRobotPack
RUN . khirobotenv.sh
CMD [ "/bin/bash"]
