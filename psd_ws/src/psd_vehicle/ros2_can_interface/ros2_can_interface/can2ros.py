#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Adjust based on your ROS 2 message types
import can
import struct
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict
from threading import Thread, Lock

class CANToROS2Subscriber(Node):
    def __init__(self, mapping_file):
        super().__init__('can_to_ros2_subscriber')
        
        # Load mapping configuration
        self.mapping = self.load_mapping(mapping_file)
        
        # Initialize CAN bus
        try:
            self.can_bus = can.interface.Bus(channel='vcan0', bustype='socketcan')
            self.get_logger().info("CAN interface 'can0' initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN interface: {e}")
            rclpy.shutdown()
            return
        
        # Create ROS 2 publishers based on the mapping
        self.publishers_list = {}
        for topic, config in self.mapping['can_to_ros'].items():
            msg_type = self.get_ros_message_type(topic)
            if msg_type is None:
                self.get_logger().warn(f"Unsupported ROS 2 message type for topic '{topic}'. Skipping.")
                continue
            self.publishers_list[topic] = self.create_publisher(msg_type, topic, 10)
            self.get_logger().info(f"Publisher created for ROS 2 topic: {topic}")
        
        # Initialize reassembly storage
        self.received_segments = defaultdict(dict)  # {can_id_base: {segment_id: data}}
        self.lock = Lock()
        
        # Start a separate thread to listen for CAN messages
        self.listener_thread = Thread(target=self.listen_can, daemon=True)
        self.listener_thread.start()

    def load_mapping(self, file):
        if not os.path.exists(file):
            self.get_logger().error(f"Mapping file '{file}' does not exist.")
            rclpy.shutdown()
            return
        with open(file, 'r') as f:
            return yaml.safe_load(f)

    def get_ros_message_type(self, topic):
        # Define mapping from topic to ROS 2 message type
        # Extend this method based on your message types
        can_to_ros_message_types = {
            '/actuator/command': Float32MultiArray,
            # Add other topic-message_type mappings here
        }
        return can_to_ros_message_types.get(topic, None)

    def listen_can(self):
        self.get_logger().info("Starting CAN listener thread.")
        while rclpy.ok():
            try:
                msg = self.can_bus.recv(timeout=1.0)  # Adjust timeout as needed
                if msg is None:
                    continue
                self.process_can_message(msg)
            except Exception as e:
                self.get_logger().error(f"Error receiving CAN message: {e}")

    def process_can_message(self, msg):
        with self.lock:
            # Determine which ROS 2 topic this CAN ID corresponds to
            for topic, config in self.mapping['can_to_ros'].items():
                can_id_base = config['can_id_base']
                # Assuming segments have incremental CAN IDs starting from can_id_base
                segment_id = msg.arbitration_id - can_id_base
                if 0 <= segment_id < len(config['segmentation']['segments']):
                    self.received_segments[can_id_base][segment_id] = msg.data
                    self.get_logger().info(f"Received CAN frame ID: {hex(msg.arbitration_id)} Data: {msg.data.hex()}")
                    
                    # Check if all segments are received
                    expected_segments = len(config['segmentation']['segments'])
                    if len(self.received_segments[can_id_base]) == expected_segments:
                        # Reassemble payload
                        payload = bytearray()
                        for idx in range(expected_segments):
                            segment_data = self.received_segments[can_id_base].get(idx)
                            if segment_data is None:
                                self.get_logger().error(f"Missing segment {idx} for CAN ID {hex(can_id_base)}")
                                return
                            payload += segment_data
                        
                        # Deserialize payload into ROS message
                        try:
                            ros_msg = self.deserialize_can_payload(payload, config['fields'])
                            # Publish the ROS message
                            self.publishers_list[topic].publish(ros_msg)
                            self.get_logger().info(f"Published ROS message on '{topic}'")
                        except Exception as e:
                            self.get_logger().error(f"Failed to deserialize CAN payload for topic '{topic}': {e}")
                        
                        # Clear the stored segments
                        del self.received_segments[can_id_base]
                else:
                    self.get_logger().warn(f"Received CAN frame ID {hex(msg.arbitration_id)} does not match any known mapping.")

    def deserialize_can_payload(self, data, fields):
        ros_msg = Float32MultiArray()
        ros_msg.data = []
        for field_name, field_config in fields.items():
            # Extract bytes for the field
            start_byte = field_config['start_bit'] // 8
            num_bytes = field_config['length'] // 8
            raw_bytes = data[start_byte:start_byte + num_bytes]
            
            # Unpack based on type and length
            fmt = self.get_struct_format(field_config['type'], field_config['length'])
            value = struct.unpack(fmt, raw_bytes)[0]
            
            ros_msg.data.append(value)
        
        return ros_msg

    def get_struct_format(self, data_type, length):
        # Define struct formats based on data type and length
        if data_type == 'float' and length == 32:
            return '<f'  # Little-endian float
        elif data_type == 'float' and length == 16:
            # Python doesn't support 16-bit floats; use custom handling or scale
            # Here, we'll treat it as unsigned short for demonstration
            return '<H'
        elif data_type == 'int' and length == 16:
            return '<H'  # Little-endian unsigned short
        elif data_type == 'int' and length == 8:
            return '<B'  # Little-endian unsigned char
        # Add more types as needed
        else:
            return '<B'  # Default to unsigned byte

def main(args=None):
    rclpy.init(args=args)
    
    # Determine the path to the mapping.yaml file
    package_share_directory = get_package_share_directory('ros2_can_interface')
    mapping_file = os.path.join(package_share_directory, 'config', 'mapping.yaml')
    
    subscriber_node = CANToROS2Subscriber(mapping_file)
    
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info("Shutting down CAN to ROS2 Subscriber Node.")
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
