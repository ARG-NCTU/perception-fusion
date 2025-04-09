REPOSITORY="argnctu/perception-fusion"
TAG="ros-humble-ros1-bridge-builder"

IMG="${REPOSITORY}:${TAG}"

docker run --rm ${IMG} | tar xvzf -