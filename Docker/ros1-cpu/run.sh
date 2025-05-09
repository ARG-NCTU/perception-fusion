#!/usr/bin/env bash

ARGS=("$@")

# project variable
PROJ_NAME="perception-fusion"

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

# Check for new_extension directory
EXTENSION_DIR=$(find /media/$USER -maxdepth 1 -type d -name "new_extension*" | head -n 1)
EXTENSION_MOUNT=""
if [ -n "$EXTENSION_DIR" ]; then
  EXTENSION_MOUNT="-v $EXTENSION_DIR:/media/arg/new_extension"
fi

docker run \
    -it \
    --rm \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e ROS_MASTER_URI=$ROS_MASTER_URI \
    -e ROS_IP=$ROS_IP \
    -e PYTHONPATH=/home/arg/$PROJ_NAME \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    -v "/home/$USER/$PROJ_NAME:/home/arg/$PROJ_NAME" \
    $EXTENSION_MOUNT \
    -w "/home/arg/$PROJ_NAME" \
    --user "root:root" \
    --name perception-fusion-ros1-cpu \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    argnctu/perception-fusion:ros1-cpu \
    $BASH_OPTION
