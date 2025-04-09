REPOSITORY="argnctu/perception-fusion"
TAG="ros-humble-ros1-bridge-builder"

IMG="${REPOSITORY}:${TAG}"

docker build . --build-arg ADD_ros_tutorials=0 -t ${IMG}