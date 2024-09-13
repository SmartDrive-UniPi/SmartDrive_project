#!/bin/bash
set -e

# Set Colors for Output
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLU='\033[37;44m'
NC='\033[0m' # No Color

# get current directory of script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# check if docker image exists
echo -e "${BLU}STEP 1: Checking if docker image exists...${NC}"
if [[ "$(docker images -q psd_noble_jazzy 2> /dev/null)" == "" ]]; then
    # build docker image
    echo -e "${ORANGE}Docker image not found. Building docker image...${NC}"
    /bin/bash $SCRIPT_DIR/build-docker.sh
else
    echo -e "${GREEN}Docker image found.${NC}"
fi

# get ros workspace path
WS_PATH=$SCRIPT_DIR/../psd_ws

# check if container has already been created
echo -e "${BLU}STEP 3: Checking docker container configuration...${NC}"
if [ -d "$WS_PATH/build" ]; then
    # run docker image
    echo -e "${GREEN}Initial setup already done.${NC}"
else
    # run first time setup
    echo -e "${ORANGE}Executing first time setup${NC}"
    # remove existing container if it exists
    # if [[ "$(docker ps -a | grep psd_container)" != "" ]]; then
    #     echo -e "${ORANGE}Removing existing container...${NC}"
    #     docker stop psd_container
    #     docker rm psd_container
    # fi
    docker run -it \
        --gpus all \
        --user ubuntu \
        --network=host \
        --ipc=host \
        -v $WS_PATH:/home/ubuntu/psd_ws \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env=DISPLAY -v /dev:/dev \
        --device-cgroup-rule="c *:* rmw" \
        --name psd_container psd_noble_jazzy \
       	/bin/bash -c "/home/ubuntu/psd_ws/deps/first_launch_script.sh"
    # commit changes to docker image
    # docker commit psd_container psd_noble_jazzy
fi

echo -e "${BLU}STEP 4: Starting docker container${NC}"
# run docker image
if [[ "$(docker ps -a | grep psd_container)" != "" ]]; then
    #echo -e "${GREEN}docker container already running...${NC}"
    echo -e "${ORANGE}Removing existing container...${NC}"
    docker stop psd_container
    docker rm psd_container
fi
xhost +local:docker
#docker start psd_container
#docker exec -it psd_container /bin/bash
docker run -it \
        --gpus all \
        --user ubuntu \
        --network=host \
        --ipc=host \
        -v $WS_PATH:/home/ubuntu/psd_ws \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --env=DISPLAY -v /dev:/dev \
        --device-cgroup-rule="c *:* rmw" \
        --name psd_container psd_noble_jazzy \
        /bin/bash
