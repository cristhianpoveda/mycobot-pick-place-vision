ARG BASE_IMAGE=ros
ARG BASE_TAG=noetic-ros-base
FROM ${BASE_IMAGE}:${BASE_TAG}

ENV DEBIAN_FRONTEND=noninteractive

COPY ./apt-requirements.txt /
RUN apt-get update \
 && xargs apt-get install -y </apt-requirements.txt \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}
RUN echo PIP_INDEX_URL=${PIP_INDEX_URL}
COPY ./py-requirements.txt /
RUN python3 -m pip install  -r /py-requirements.txt

# create user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} mycobot
RUN adduser --gecos "MYCOBOT User" --disabled-password --uid ${UID} --gid ${GID} mycobot
RUN usermod -a -G dialout mycobot
ADD config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

ENV USER mycobot
USER mycobot

ENV HOME /home/${USER}
RUN mkdir -p ${HOME}/catkin_ws/src
WORKDIR ${HOME}/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace"

COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown mycobot /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

COPY config/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh; sudo chown mycobot /entrypoint.sh ;

RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/* 
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
