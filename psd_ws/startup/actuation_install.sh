#!/bin/bash

# 0) Change to the project root directory
cd ~/SmarDrive_project/psd_ws || { echo "Project directory not found! Exiting."; exit 1; }

# 1) Update rosdep and install dependencies
echo "Updating rosdep and installing dependencies..."
rosdep update && rosdep install --from-paths src -i -y
if [ $? -ne 0 ]; then
    echo "rosdep encountered an error. Exiting."
    exit 1
fi

# 2) Find the configuration file and prompt user for editing
echo "Searching for vesc_config.yaml..."
config_file=$(find src -type f -path "*/vesc/vesc_driver/params/vesc_config.yaml" | head -n 1)

if [ -z "$config_file" ]; then
    echo "Configuration file 'vesc_config.yaml' not found in the src folder."
else
    echo "Found configuration file at: $config_file"
    echo "-------------------------"
    cat "$config_file"
    echo "-------------------------"
    read -p "Do you want to edit this file? (y/N): " answer
    case "$answer" in
        [yY])
            nano "$config_file"
            ;;
        *)
            echo "Proceeding without editing the file."
            ;;
    esac
fi

# 3) Build the workspace using colcon
echo "Building the workspace with colcon..."
colcon build --symlink-install
if [ $? -ne 0 ]; then
    echo "colcon build failed. Please check the errors above."
    exit 1
fi

# 4) Launch the ros2 node
echo "Launching the ros2 node..."
ros2 launch vesc_driver vesc_driver_node.launch.py

# 4.1) If a permission error on the serial port occurs,
# remind the user to change permissions.
echo ""
echo "If you encounter a 'permission denied' error on the serial port, please run:"
echo "sudo chmod 777 /dev/ttyACM0"
