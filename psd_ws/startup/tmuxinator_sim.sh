#!/bin/bash
set -e

# Get the absolute path of the script and navigate to its directory
SCRIPT=$(readlink -f "$0")
cd "$(dirname "$SCRIPT")"

# Perform a colcon build in the ~/psd_ws/ directory
echo "Running colcon build in ~/psd_ws/"
cd ~/psd_ws/
colcon build --symlink-install --parallel-workers $(nproc)
echo "colcon build completed"

# start tmuxinator
cd "$(dirname "$SCRIPT")"
tmuxinator start -p session_custom_sim.yml 
