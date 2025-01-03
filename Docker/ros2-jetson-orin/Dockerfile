FROM dustynv/ros:humble-desktop-l4t-r35.3.1

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
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends \
    git \
    zip \
    pip \
    vim \
    curl \
    wget \
    sudo \
    cmake \
    net-tools \
    lsb-release \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

#################################### realsense manual installation ####################################

# Manual Installation of librealsense SDK
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    apt-get update && \
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    sudo apt-get install -y librealsense2-utils librealsense2-dev

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> /etc/bash.bashrc
RUN echo "arg ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]
