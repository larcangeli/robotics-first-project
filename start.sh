#!/bin/bash

# Specify the container name and image
CONTAINER_NAME="robotics_hw"
IMAGE_NAME="smentasti/robotics"

# Pull the latest image
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME

# Grant X permissions for GUI applications (if needed)
xhost +si:localuser:$(whoami)

# Check if the container exists
if docker ps -a | grep -q $CONTAINER_NAME; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f {{.State.Running}} $CONTAINER_NAME)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

# Ensure the local 'data' folder exists
mkdir -p "$(pwd)/data"
mkdir -p "$(pwd)/catkin_ws"

# Run the container
docker run -it \
    --privileged \
    --user robotics \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    --rm \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/data:/home/robotics/data" \
    --volume="$(pwd)/catkin_ws:/home/robotics/catkin_ws" \
    --name $CONTAINER_NAME \
    -w /home/robotics \
    $IMAGE_NAME
