FROM ubuntu:20.04

SHELL [ "/bin/bash", "-c" ]

RUN apt update && apt -y upgrade
RUN apt-get update && apt -y upgrade
RUN apt install -y sudo git curl wget make tmux x11-apps

# Install Python 3.7.16
# RUN bash <(curl -sSL https://raw.githubusercontent.com/skrjtech/linux-setup/main/installer/pycharm.sh) 3.7.16
# ENV PATH /usr/local/python/bin:$PATH

# Ros Melodic Install
RUN git clone https://github.com/skrjtech/rosBeginner.git
RUN cd rosBeginner && . build.sh && cd ..
RUN rm -rf rosBeginner

COPY archives/khirobotenv.sh /khirobotenv.sh
COPY archives/khiRobotPack /khiRobotPack

CMD [ "/bin/bash"]