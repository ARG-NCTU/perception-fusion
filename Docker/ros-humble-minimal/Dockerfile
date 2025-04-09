# Use the official ROS Humble base image
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO=humble

###################################### user #####################################

ENV SHELL=/bin/bash \
    USER=arg \
    UID=1000 \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8

ENV HOME=/home/${USER}

RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER} 

RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd


###################################### basic #####################################
# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-dev-tools \
    libboost-all-dev \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \ 
    openssh-server \
    libopencv-dev\
    pyqt5-dev \
    python3-pandas \
    ros-${ROS_DISTRO}-geodesy\
    ros-${ROS_DISTRO}-pcl-conversions 

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "arg ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]
