FROM dustynv/ros:noetic-ros-base-l4t-r35.3.1

ARG DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=rolling

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


###################################### basic tools #####################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    python3-pip \
    sudo \
    git \
    wget \
    vim \
    wget \
    cmake \
    git \
    libbullet-dev \
    libpython3-dev \
    python3-empy \
    python3-numpy \
    python3-pytest \
    python3-yaml \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    apt-utils \
    && apt-get update && apt-get install -y git \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

##################################### ROS1-ROS2-Bridge ################################

# RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list' && \
#     apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS1_DISTRO}-catkin \
#     ros-${ROS1_DISTRO}-roscpp \
#     ros-${ROS1_DISTRO}-roslaunch \
#     ros-${ROS1_DISTRO}-rosmsg \
#     ros-${ROS1_DISTRO}-std-msgs \
#     python3-rospkg \
#     ros-${ROS1_DISTRO}-rosbash \
#     ros-${ROS1_DISTRO}-roscpp-tutorials \
#     ros-${ROS1_DISTRO}-rospy-tutorials \
#     ros-${ROS1_DISTRO}-rostopic \
#     ros-${ROS1_DISTRO}-rqt-image-view && \
#     apt-get clean && rm -rf /var/lib/apt/lists/*

###################################### ROS 2 Humble #####################################

# RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
#     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
#     sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
#     apt-get update && apt-get install -y --no-install-recommends \
#     ros-${ROS2_DISTRO}-desktop \
#     python3-${ROS2_DISTRO}-common-extensions \
#     python3-rosdep && \
#     rosdep init && rosdep update && \
#     apt-get clean && rm -rf /var/lib/apt/lists/*

######################################## pip ########################################
RUN python3 -m pip install --upgrade \
    importlib_metadata \
    'setuptools<70.0' \
    pytest

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS1_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "source /opt/ros/${ROS2_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "arg ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]