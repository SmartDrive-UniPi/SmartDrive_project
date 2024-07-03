#!/bin/bash
set -e

# Build the docker image
# get current directory of script
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR/../psd_docker/
# build docker image
echo "Building docker image"
docker build -t psd_noble_jazzy .
echo "Done"
