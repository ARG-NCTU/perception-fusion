FROM osrf/ros:foxy-desktop

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=foxy

###################################### ROS1 ######################################
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS1_DISTRO}-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-catkin-tools && \
    if [ -e /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rm /etc/ros/rosdep/sources.list.d/20-default.list; \
    fi && \
    rosdep init && rosdep update && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

###################################### ROS1-ROS2-Bridge ####################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS2_DISTRO}-ros1-bridge && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
    
##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS1_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /etc/bash.bashrc

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]