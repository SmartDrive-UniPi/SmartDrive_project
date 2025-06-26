# 1. First, install Python dependencies
cd ~/ros2_ws/src/psd_perception_camera
./scripts/install_dependencies.sh

# 2. Build the package
cd ~/ros2_ws
colcon build --packages-select psd_perception_camera --symlink-install

# 3. Source the workspace
source install/setup.bash

# 4. Run with default parameters
ros2 launch psd_perception_camera cone_detection.launch.py

# 5. Run with custom engine path
ros2 launch psd_perception_camera cone_detection.launch.py \
  engine_path:=/path/to/your/model.engine

# 6. Run without RViz
ros2 launch psd_perception_camera cone_detection.launch.py \
  engine_path:=/path/to/your/model.engine \
  use_rviz:=false

# 7. Run with custom parameters file
ros2 launch psd_perception_camera cone_detection.launch.py \
  params_file:=/path/to/custom_params.yaml