
# ROS2 CAN interface

`ros2_can_interface` is a ROS 2 package designed to facilitate seamless communication between ROS 2 topics and Controller Area Network (CAN) messages. It provides two primary nodes:

1. **ROS2 to CAN Publisher (`ros2can`)**: Subscribes to specified ROS 2 topics, serializes the messages, segments them if necessary, and transmits them over the CAN bus.

2. **CAN to ROS2 Subscriber (`can2ros`)**: Listens for CAN messages, reassembles segmented messages, deserializes them, and publishes the data to corresponding ROS 2 topics.

This package is particularly useful in robotics and automotive applications where integrating ROS 2 systems with CAN-based hardware is required.

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
  - [Building the Package](#building-the-package)
  - [Running the Nodes](#running-the-nodes)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Features

- **Bidirectional Communication**: Convert ROS 2 messages to CAN frames and vice versa.
- **Custom Segmentation**: Automatically segment large messages into multiple CAN frames based on configuration.
- **Flexible Configuration**: Define mappings between ROS 2 topics and CAN messages using a YAML configuration file.
- **Threaded CAN Listener**: Efficiently listen to CAN bus messages without blocking ROS 2 operations.
- **Logging**: Comprehensive logging for monitoring and debugging.

---

## Prerequisites

- **Operating System**: Ubuntu 20.04 or later (compatible with ROS 2 distributions like Foxy, Galactic, etc.)
- **ROS 2**: Installed and properly sourced.
- **Python 3**: Version 3.8 or later.
- **CAN Interface Hardware**: Such as USB-CAN adapters. Alternatively, use a virtual CAN interface (`vcan`) for testing.

---

## Installation

1. **Set Up Your ROS 2 Workspace**:

    ```bash
    cd ~/psd_ws/src
    ```

2. **Clone the Package**:

    ```bash
    git clone https://github.com/rimaturus/ros2_can_interface.git
    ```

3. **Install Python Dependencies**:

    ```bash
    pip install python-can pyyaml ament_index_python --break-system-packages 
    ```
    **Achtung!** [WIP cleaner fix] 

4. **Build the Package**:

    ```bash
    cd ~/psd_ws/
    colcon build --packages-select ros2_can_interface
    ```

5. **Source the Workspace**:

    ```bash
    source install/setup.bash
    ```

---

## Configuration

### `mapping.yaml`

The `mapping.yaml` file defines how ROS 2 topics map to CAN messages and vice versa. It specifies CAN IDs, message fields, data types, and segmentation parameters.

**Location:** `ros2_can_interface/config/mapping.yaml`

**Sample `mapping.yaml`:**

```yaml
ros_to_can:
  /psd_vehicle/pose:
    can_id_base: 0x200
    fields:
      car_x: 
        start_bit: 0
        length: 32
        type: float
      car_y:
        start_bit: 32
        length: 32
        type: float
      car_yaw:
        start_bit: 64
        length: 32
        type: float
    segmentation:
      max_payload: 8  # bytes per CAN frame
      segments:
        - offset: 0  # bytes
        - offset: 4
        - offset: 8

can_to_ros:
  /actuator/command:
    can_id_base: 0x200
    fields:
      command:
        start_bit: 0
        length: 16
        type: int
      value:
        start_bit: 16
        length: 32
        type: float
    segmentation:
      max_payload: 8  # bytes per CAN frame
      segments:
        - offset: 0
        - offset: 4
```

---

## Usage

### Building the Package

If you've made changes to the package or added new dependencies, rebuild the workspace:

```bash
cd ~/psd_ws/
colcon build --packages-select ros2_can_interface
source install/setup.bash
```

### Running the Nodes

#### 1. Run the ROS2 to CAN Publisher Node (`ros2can`)

This node subscribes to ROS 2 topics and publishes corresponding CAN messages.

```bash
ros2 run ros2_can_interface ros2can
```

#### 2. Run the CAN to ROS2 Subscriber Node (`can2ros`)

This node listens to CAN messages and publishes corresponding ROS 2 topics.

```bash
ros2 run ros2_can_interface can2ros
```

---

## Testing

### Using Virtual CAN (`vcan`) for Testing

1. **Set Up Virtual CAN Interface:**

   ```bash
   sudo modprobe vcan
   sudo ip link add dev vcan0 type vcan
   sudo ip link set up vcan0
   ```

2. **Monitor CAN Traffic:**

   Use `candump` to monitor CAN messages:

   ```bash
   candump vcan0
   ```

3. **Send Test CAN Messages:**

   Use `cansend` to send test messages:

   ```bash
   cansend vcan0 100#1122334455667788
   ```

4. **Publish Test ROS 2 Messages:**

   In a new terminal, publish a test message:

   ```bash
   ros2 topic pub /psd_vehicle/pose std_msgs/msg/Float32MultiArray "data: [25.5, 3.2, -1.1]"
   ```

5. **Verify Reception:**

   Echo the ROS 2 subscriber topic:

   ```bash
   ros2 topic echo /actuator/command
   ```

---

## Troubleshooting

### Common Issues and Solutions

- **`ModuleNotFoundError: No module named 'ros2_can_interface.ros2can'`**:

  Ensure that `ros2can.py` is correctly placed inside the `ros2_can_interface` package and `setup.py` is correctly configured.

- **`AttributeError: property 'subscriptions' of 'ROS2ToCANPublisher' object has no setter`**:

  Rename `self.subscriptions` to `self.subscription_list` or another non-conflicting name in your scripts.

- **CAN Bus Not Initializing**:

  Verify the CAN interface name and permissions. Add your user to the `can` group:

  ```bash
  sudo usermod -aG can $USER
  ```

---

## Package Structure

```
ros2_can_interface/
├── config
│   └── mapping.yaml
├── package.xml
├── README.md
├── resource
│   └── ros2_can_converter
├── ros2_can_converter
│   ├── can2ros.py
│   ├── __init__.py
│   └── ros2can.py
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

---

## Dependencies

- **ROS 2 Packages**: `rclpy`, `std_msgs`
- **Python Packages**: `python-can`, `pyyaml`, `ament_index_python`
- **System Packages**: `can-utils`

---

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork the Repository**.
2. **Create a Branch**.
3. **Commit Changes**.
4. **Open a Pull Request**.

---

## License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- **ROS 2 Community**
- **Python CAN Library Authors**
- **Open-Source Contributors**

---

*Happy Coding! 🚀*
