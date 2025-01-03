FROM nvidia/cuda:12.0.0-cudnn8-devel-ubuntu22.04

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
    ccache \
    g++ \
    gcc \
    gdb \
    gawk \
    make \
    ninja-build \
    python3-numpy \
    python3-dev \
    python3-pyparsing \
    python3-serial \
    python-is-python3 \
    libpython3-stdlib \
    libtool-bin \
    zip \
    default-jre \
    socat \
    tzdata \
    pip \
    dirmngr \
    gnupg2 \
    git \
    gitk git-gui \
    vim \
    less \
    curl \
    wget \
    sudo \
    cmake \
    swig \
    net-tools \
    lsb-release \
    gnupg \
    build-essential \
    libcgal-dev \
    libfftw3-dev \
    rapidjson-dev \
    pkg-config \
    libboost-system-dev \
    libopencv-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

###################################### ROS2 ####################################

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-image-transport-plugins \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-dev-tools && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

######################################## realsense-ros2 ########################################
RUN python3 -m pip install --upgrade \
    importlib_metadata \
    'setuptools<70.0' \
    pytest

##################################### bag to tools ######################################

RUN python3 -m pip install --upgrade \
    gpxpy \
    tqdm 

###################################### cv ################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-opencv 

RUN python3 -m pip install --upgrade \
    opencv-python

#################################### pointcloud_processing ###################################

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-ros \
    python3-numpy \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN sudo -H pip install transforms3d

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "arg ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]
