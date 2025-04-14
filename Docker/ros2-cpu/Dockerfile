FROM ubuntu:22.04

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

# RUN apt-get update && apt-get install -y --no-install-recommends \
#     python3-opencv 

# RUN python3 -m pip install --upgrade \
#     opencv-python

RUN add-apt-repository universe && apt-get update && \
    apt-get install -y --no-install-recommends \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-tools gstreamer1.0-libav gstreamer1.0-gl \
    gstreamer1.0-gtk3 gstreamer1.0-x libgstreamer1.0-0 \
    libgstrtspserver-1.0-0 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

ENV OPENCV_VERSION=4.5.2
ARG MTX_VERSION=1.11.3

# --------------------- 下載 & 編譯 OpenCV ---------------------
RUN git clone -b ${OPENCV_VERSION} --depth 1 https://github.com/opencv/opencv.git /opencv && \
    git clone -b ${OPENCV_VERSION} --depth 1 https://github.com/opencv/opencv_contrib.git /opencv_contrib && \
    mkdir -p /opencv/build

WORKDIR /opencv/build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/opencv_contrib/modules \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D PYTHON_EXECUTABLE=/usr/bin/python3 \
          -D OPENCV_GENERATE_PKGCONFIG=YES \
          -D BUILD_opencv_python2=OFF \
          -D BUILD_opencv_python3=ON \
          -D WITH_GSTREAMER=ON \
          -D WITH_FFMPEG=ON \
          -D WITH_V4L=ON \
          .. && \
    make -j"$(nproc)" && make install && ldconfig


RUN python3 -c "import cv2; print('OpenCV:', cv2.__version__, 'GStreamer:', cv2.getBuildInformation().find('GStreamer'))"

# --------------------- 安裝 Python 擴充 ---------------------
RUN pip3 install tqdm websocket-server websocket-client ffmpeg-python

# --------------------- 安裝 FFmpeg 播放支援 ---------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libavcodec-dev libavformat-dev libavutil-dev \
    libswscale-dev libavfilter-dev libavdevice-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --------------------- 安裝 nginx + RTMP 模組 ---------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    nginx vim libnginx-mod-rtmp && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --------------------- 安裝 MediaMTX ---------------------
RUN mkdir -p /opt/mediamtx && \
    curl -L https://github.com/bluenviron/mediamtx/releases/download/v${MTX_VERSION}/mediamtx_v${MTX_VERSION}_linux_amd64.tar.gz \
    | tar xz -C /opt/mediamtx && \
    ln -s /opt/mediamtx/mediamtx /usr/local/bin/mediamtx

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

##################################### cyclonedds ######################################

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

########################################## velodyne ####################################

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-velodyne* \
    && rm -rf /var/lib/apt/lists/*

###################################### ros2 bag processing ################################

RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pybind11

##################################### setting ##########################################

RUN chown -R ${USER}:${USER} ${HOME}/
RUN echo "${USER} ALL=(ALL)  ALL" >> /etc/sudoers
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc
RUN echo "arg ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER}
WORKDIR ${HOME}
CMD ["/bin/bash"]
