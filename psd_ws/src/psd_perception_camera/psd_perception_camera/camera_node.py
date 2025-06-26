#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from sklearn.cluster import DBSCAN
import struct
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.msg import ParameterDescriptor

# Custom message for cone detections (create this in your package)
# You can also use visualization_msgs/MarkerArray if you prefer
from geometry_msgs.msg import PointStamped


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('engine_path', 'path/to/your/model.engine',
                              ParameterDescriptor(description='Path to TensorRT engine file'))
        self.declare_parameter('conf_threshold', 0.5,
                              ParameterDescriptor(description='Confidence threshold for detection'))
        self.declare_parameter('nms_threshold', 0.4,
                              ParameterDescriptor(description='NMS threshold'))
        self.declare_parameter('input_width', 640,
                              ParameterDescriptor(description='Model input width'))
        self.declare_parameter('input_height', 640,
                              ParameterDescriptor(description='Model input height'))
        self.declare_parameter('dbscan_eps', 0.1,
                              ParameterDescriptor(description='DBSCAN clustering epsilon'))
        self.declare_parameter('dbscan_min_samples', 5,
                              ParameterDescriptor(description='DBSCAN minimum samples'))
        
        # Get parameters
        self.engine_path = self.get_parameter('engine_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.dbscan_eps = self.get_parameter('dbscan_eps').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        
        # Initialize TensorRT
        self.trt_logger = trt.Logger(trt.Logger.WARNING)
        self.engine = self.load_engine(self.engine_path)
        self.context = self.engine.create_execution_context()
        
        # Allocate buffers
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Storage for latest data
        self.latest_image = None
        self.latest_pointcloud = None
        self.latest_image_msg = None  # Store original message for header info
        
        # Define cone class colors (adjust based on your FSOCO classes)
        self.class_colors = {
            0: (255, 0, 0),      # Blue cone
            1: (255, 255, 0),    # Yellow cone
            2: (0, 165, 255),    # Orange cone
            3: (128, 0, 128),    # Purple cone (large)
            # Add more classes as needed
        }
        self.class_names = {
            0: "Blue",
            1: "Yellow", 
            2: "Orange",
            3: "Large",
            # Add more classes as needed
        }
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/zed2/zed_node/left/image_rect_color',  # Adjust topic name as needed
            self.image_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/zed2/zed_node/point_cloud/cloud_registered',  # Adjust topic name as needed
            self.pointcloud_callback,
            10
        )
        
        # Publishers
        self.cone_publisher = self.create_publisher(
            PointCloud2,
            '/possible_cones',
            10
        )
        
        # Publisher for bounding box visualization
        self.bbox_image_publisher = self.create_publisher(
            Image,
            '/detected_bb',
            10
        )
        
        self.get_logger().info('Cone Detector Node initialized')
        self.get_logger().info(f'Publishing bounding box visualization to /detected_bb')
    
    def load_engine(self, engine_path):
        """Load TensorRT engine from file"""
        with open(engine_path, 'rb') as f:
            runtime = trt.Runtime(self.trt_logger)
            return runtime.deserialize_cuda_engine(f.read())
    
    def allocate_buffers(self):
        """Allocate CUDA buffers for TensorRT inference"""
        inputs = []
        outputs = []
        bindings = []
        stream = cuda.Stream()
        
        for i in range(self.engine.num_io_tensors):
            tensor_name = self.engine.get_tensor_name(i)
            size = trt.volume(self.engine.get_tensor_shape(tensor_name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(tensor_name))
            
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            
            # Append the device buffer address to device bindings
            bindings.append(int(device_mem))
            
            # Append to the appropriate input/output list
            if self.engine.get_tensor_mode(tensor_name) == trt.TensorIOMode.INPUT:
                inputs.append({'host': host_mem, 'device': device_mem})
            else:
                outputs.append({'host': host_mem, 'device': device_mem})
        
        return inputs, outputs, bindings, stream
    
    def preprocess_image(self, image):
        """Preprocess image for YOLOv11"""
        # Resize image
        resized = cv2.resize(image, (self.input_width, self.input_height))
        
        # Convert BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        normalized = rgb.astype(np.float32) / 255.0
        
        # Transpose to CHW format
        transposed = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        batched = np.expand_dims(transposed, axis=0)
        
        return batched.astype(np.float32)
    
    def postprocess_detections(self, output, original_shape):
        """Post-process YOLOv11 detections"""
        # YOLOv11 output format: [batch, num_detections, 85] for COCO
        # For FSOCO it might be different, adjust accordingly
        # Assuming output shape is [1, num_detections, 6] where 6 = [x, y, w, h, conf, class]
        
        detections = []
        
        # Get original image dimensions
        orig_h, orig_w = original_shape[:2]
        
        # Scale factors
        scale_x = orig_w / self.input_width
        scale_y = orig_h / self.input_height
        
        # Process detections
        for detection in output[0]:
            x_center, y_center, width, height, confidence, class_id = detection[:6]
            
            if confidence > self.conf_threshold:
                # Convert to corner format
                x1 = int((x_center - width / 2) * scale_x)
                y1 = int((y_center - height / 2) * scale_y)
                x2 = int((x_center + width / 2) * scale_x)
                y2 = int((y_center + height / 2) * scale_y)
                
                # Clip to image boundaries
                x1 = max(0, min(x1, orig_w - 1))
                y1 = max(0, min(y1, orig_h - 1))
                x2 = max(0, min(x2, orig_w - 1))
                y2 = max(0, min(y2, orig_h - 1))
                
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': confidence,
                    'class_id': int(class_id)
                })
        
        # Apply NMS
        if detections:
            boxes = np.array([d['bbox'] for d in detections])
            scores = np.array([d['confidence'] for d in detections])
            
            indices = cv2.dnn.NMSBoxes(
                boxes.tolist(),
                scores.tolist(),
                self.conf_threshold,
                self.nms_threshold
            )
            
            if len(indices) > 0:
                indices = indices.flatten()
                detections = [detections[i] for i in indices]
        
        return detections
    
    def run_inference(self, image):
        """Run TensorRT inference"""
        # Preprocess image
        input_data = self.preprocess_image(image)
        
        # Copy input to device
        np.copyto(self.inputs[0]['host'], input_data.ravel())
        cuda.memcpy_htod_async(self.inputs[0]['device'], self.inputs[0]['host'], self.stream)
        
        # Run inference
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        
        # Copy output to host
        cuda.memcpy_dtoh_async(self.outputs[0]['host'], self.outputs[0]['device'], self.stream)
        
        # Synchronize
        self.stream.synchronize()
        
        # Reshape output
        output_shape = self.engine.get_binding_shape(1)  # Assuming output is at index 1
        output = self.outputs[0]['host'].reshape(output_shape)
        
        return output
    
    def extract_points_from_bbox(self, pointcloud, bbox, image_width, image_height):
        """Extract 3D points within a bounding box from point cloud"""
        x1, y1, x2, y2 = bbox
        points_in_bbox = []
        
        # Read point cloud data
        for point in pc2.read_points(pointcloud, field_names=['x', 'y', 'z', 'rgb'], skip_nans=True):
            x, y, z, rgb = point
            
            # Project 3D point to image plane
            # This is a simplified projection - adjust based on your camera parameters
            # For ZED2, you might need to use the camera intrinsics
            # Assuming the point cloud is already aligned with the image
            
            # Get pixel coordinates (this depends on your pointcloud format)
            # If the pointcloud includes UV coordinates, use those
            # Otherwise, you'll need to project using camera intrinsics
            
            # Simplified approach assuming organized point cloud
            # You might need to adjust this based on your actual setup
            u = int((x / z) * image_width / 2 + image_width / 2)
            v = int((y / z) * image_height / 2 + image_height / 2)
            
            # Check if point is within bounding box
            if x1 <= u <= x2 and y1 <= v <= y2:
                points_in_bbox.append([x, y, z])
        
        return np.array(points_in_bbox)
    
    def cluster_points(self, points):
        """Cluster 3D points using DBSCAN"""
        if len(points) < self.dbscan_min_samples:
            # If too few points, return the mean as single cluster
            return [np.mean(points, axis=0)]
        
        # Perform DBSCAN clustering
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        
        # Extract cluster centroids
        centroids = []
        unique_labels = set(clustering.labels_)
        
        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue
            
            cluster_points = points[clustering.labels_ == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)
        
        return centroids
    
    def draw_bounding_boxes(self, image, detections):
        """Draw bounding boxes with labels on the image"""
        # Create a copy to avoid modifying the original
        viz_image = image.copy()
        
        for detection in detections:
            bbox = detection['bbox']
            confidence = detection['confidence']
            class_id = detection['class_id']
            
            # Get color and name for this class
            color = self.class_colors.get(class_id, (0, 255, 0))  # Default to green
            class_name = self.class_names.get(class_id, f"Class_{class_id}")
            
            # Draw bounding box
            cv2.rectangle(viz_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 3)
            
            # Create label with class name and confidence
            label = f"{class_name}: {confidence:.2f}"
            
            # Calculate label position and background
            label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            label_y = bbox[1] - 10 if bbox[1] - 10 > label_size[1] else bbox[1] + label_size[1] + 10
            
            # Draw label background
            cv2.rectangle(viz_image, 
                         (bbox[0], label_y - label_size[1] - 5),
                         (bbox[0] + label_size[0], label_y + 5),
                         color, -1)
            
            # Draw label text
            cv2.putText(viz_image, label, 
                       (bbox[0], label_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 255), 2)
        
        # Add detection count
        info_text = f"Detected {len(detections)} cone(s)"
        cv2.putText(viz_image, info_text, 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1.0, (0, 255, 0), 2)
        
        return viz_image
    
    def image_callback(self, msg):
        """Handle incoming image messages"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.latest_image_msg = msg  # Store the message for header info
        
        # Process if we have both image and pointcloud
        if self.latest_image is not None and self.latest_pointcloud is not None:
            self.process_detection()
    
    def pointcloud_callback(self, msg):
        """Handle incoming pointcloud messages"""
        self.latest_pointcloud = msg
        
        # Process if we have both image and pointcloud
        if self.latest_image is not None and self.latest_pointcloud is not None:
            self.process_detection()
    
    def process_detection(self):
        """Main processing pipeline"""
        # Run cone detection
        output = self.run_inference(self.latest_image)
        detections = self.postprocess_detections(output, self.latest_image.shape)
        
        # Draw bounding boxes on visualization image
        viz_image = self.draw_bounding_boxes(self.latest_image, detections)
        
        # Extract 3D points for each detection
        all_centroids = []
        
        for detection in detections:
            bbox = detection['bbox']
            
            # Extract points within bbox
            points = self.extract_points_from_bbox(
                self.latest_pointcloud, bbox, 
                self.latest_image.shape[1], self.latest_image.shape[0]
            )
            
            if len(points) > 0:
                # Cluster points and get centroids
                centroids = self.cluster_points(points)
                all_centroids.extend(centroids)
        
        # Publish centroids as PointCloud2
        if all_centroids:
            self.publish_centroids(all_centroids)
        
        # Publish visualization image with bounding boxes
        bbox_msg = self.bridge.cv2_to_imgmsg(viz_image, 'bgr8')
        bbox_msg.header = self.latest_image_msg.header  # Use original image header
        self.bbox_image_publisher.publish(bbox_msg)
        
        # Log detection info
        if detections:
            self.get_logger().debug(f'Detected {len(detections)} cones, published to /detected_bb')
    
    def publish_centroids(self, centroids):
        """Publish cone centroids as PointCloud2"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.latest_pointcloud.header.frame_id
        
        # Create PointCloud2 message
        points = []
        for centroid in centroids:
            x, y, z = centroid
            # Pack as float32
            points.append(struct.pack('fff', x, y, z))
        
        # Create PointCloud2
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(centroids)
        cloud.fields = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = b''.join(points)
        
        self.cone_publisher.publish(cloud)
        self.get_logger().info(f'Published {len(centroids)} cone centroids')


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()