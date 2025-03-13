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

class ROS2ToCANPublisher(Node):
    def __init__(self, mapping_file):
        super().__init__('ros2_to_can_publisher')
        
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
        
        # Create ROS 2 subscriptions based on the mapping
        self.subscriptions_list = []
        for topic, config in self.mapping['ros_to_can'].items():
            msg_type = self.get_ros_message_type(topic)
            if msg_type is None:
                self.get_logger().warn(f"Unsupported ROS 2 message type for topic '{topic}'. Skipping.")
                continue
            subscription = self.create_subscription(
                msg_type,
                topic,
                self.create_callback(config),
                10
            )
            self.subscriptions_list.append(subscription)
            self.get_logger().info(f"Subscribed to ROS 2 topic: {topic}")

    def load_mapping(self, file):
        if not os.path.exists(file):
            self.get_logger().error(f"Mapping file '{file}' does not exist.")
            rclpy.shutdown()
        with open(file, 'r') as f:
            return yaml.safe_load(f)

    def get_ros_message_type(self, topic):
        # Define mapping from topic to ROS 2 message type
        # Extend this method based on your message types
        ros_to_can_message_types = {
            '/sensor/temperature_velocity': Float32MultiArray,
            # Add other topic-message_type mappings here
        }
        return ros_to_can_message_types.get(topic, None)

    def create_callback(self, config):
        def callback(msg):
            try:
                # Serialize the ROS message into a bytearray
                payload = self.serialize_ros_message(msg, config['fields'])
                
                # Segment the payload if necessary
                segments = self.segment_payload(payload, config['segmentation']['max_payload'])
                
                # Send each segment with incremental CAN IDs
                can_id_base = config['can_id_base']
                for idx, segment in enumerate(segments):
                    can_id = can_id_base + idx
                    can_msg = can.Message(arbitration_id=can_id, data=segment, is_extended_id=False)
                    self.can_bus.send(can_msg)
                    self.get_logger().info(f"Sent CAN frame ID: {hex(can_id)} Data: {segment.hex()}")
            except Exception as e:
                self.get_logger().error(f"Failed to send CAN message: {e}")
        return callback

    def serialize_ros_message(self, msg, fields):
        # Initialize a bytearray with the required size
        # Calculate the total bits needed
        total_bits = max(field['start_bit'] + field['length'] for field in fields.values())
        total_bytes = (total_bits + 7) // 8
        payload = bytearray(total_bytes)
        
        for field_name, field_config in fields.items():
            # Extract the value from the ROS message
            if isinstance(msg, Float32MultiArray):
                # Assuming data is a list and field order matches
                value = msg.data.pop(0) if msg.data else 0.0
            else:
                value = getattr(msg, field_name, 0)
            
            # Pack the value based on its type and length
            fmt = self.get_struct_format(field_config['type'], field_config['length'])
            packed = struct.pack(fmt, value)
            
            # Insert the packed bytes into the payload at the specified position
            start_byte = field_config['start_bit'] // 8
            end_byte = start_byte + len(packed)
            payload[start_byte:end_byte] = packed
        
        return payload

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

    def segment_payload(self, payload, max_payload):
        return [payload[i:i + max_payload] for i in range(0, len(payload), max_payload)]

def main(args=None):
    rclpy.init(args=args)
    
    # Determine the path to the mapping.yaml file
    package_share_directory = get_package_share_directory('ros2_can_interface')
    mapping_file = os.path.join(package_share_directory, 'config', 'mapping.yaml')
    
    publisher_node = ROS2ToCANPublisher(mapping_file)
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info("Shutting down ROS2 to CAN Publisher Node.")
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
