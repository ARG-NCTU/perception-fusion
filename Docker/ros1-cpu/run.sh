#!/usr/bin/env bash

ARGS=("$@")

# project variable
PROJ_NAME="perception-fusion"

# Specify dataset, log, and model directories
HOST_DATA_DIR="~/perception-fusion/data/nuscenes"
HOST_LOG_DIR="~/perception-fusion/crfnet/tb_logs"
HOST_MODEL_DIR="~/perception-fusion/crfnet/saved_models"

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

docker run \
    -it \
    --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e HOST_DATA_DIR=$HOST_DATA_DIR \
    -e HOST_LOG_DIR=$HOST_LOG_DIR \
    -e HOST_MODEL_DIR=$HOST_MODEL_DIR \
    -e PYTHONPATH=/home/arg/$PROJ_NAME \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    -v "/home/$USER/$PROJ_NAME:/home/arg/$PROJ_NAME" \
    -v "/media/$USER/new_extension/bags:/media/arg/new_extension/bags" \
    -w "/home/arg/$PROJ_NAME" \
    --user "root:root" \
    --name perception-fusion-ros1-cpu \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    argnctu/perception-fusion:ros1-cpu \
    $BASH_OPTION
