#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>
#include <chrono>
// Add OpenMP header
#include <omp.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Core>

class PointCloudSubscriber : public rclcpp::Node
{
public:
  PointCloudSubscriber() : Node("pointcloud_subscriber")
  {
    // Declare and get parameter to bypass ground removal
    this->declare_parameter<bool>("bypass_ground_removal", false);
    bypass_ground_removal_ = this->get_parameter("bypass_ground_removal").as_bool();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed/zed_node/depth/points", 10,
      std::bind(&PointCloudSubscriber::pointcloud_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/markers", 10);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Filter out invalid points and those with norm < 0.5
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & pt : cloud->points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        float norm = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (norm >= 0.5f) {
          filtered_cloud->points.push_back(pt);
        }
      }
    }
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    if (filtered_cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid points found");
      return;
    }

    // If bypass_ground_removal_ is true, skip ground removal, else remove ground plane using RANSAC segmentation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
    if (bypass_ground_removal_) {
      cloud_no_ground = filtered_cloud;
      RCLCPP_INFO(this->get_logger(), "Bypassing ground removal");
    } else {
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.02);
      seg.setMaxIterations(1000);
      seg.setInputCloud(filtered_cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      }

      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(filtered_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);  // Remove ground
      extract.filter(*cloud_no_ground);
    }

    // Compute Z percentiles (10th and 99th) from cloud_no_ground
    std::vector<float> z_values;
    for (const auto & pt : cloud_no_ground->points) {
      z_values.push_back(pt.z);
    }
    if (z_values.empty()) {
      RCLCPP_WARN(this->get_logger(), "No points after ground removal or filtering");
      return;
    }
    std::sort(z_values.begin(), z_values.end());
    size_t idx_lower = static_cast<size_t>(0.10 * (z_values.size() - 1));
    size_t idx_upper = static_cast<size_t>(0.99 * (z_values.size() - 1));
    float z_lower_bound = z_values[idx_lower];
    float z_upper_bound = z_values[idx_upper];

    // Filter points by z range
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & pt : cloud_no_ground->points) {
      if (pt.z >= z_lower_bound && pt.z <= z_upper_bound) {
        cloud_z_filtered->points.push_back(pt);
      }
    }
    cloud_z_filtered->width = cloud_z_filtered->points.size();
    cloud_z_filtered->height = 1;
    cloud_z_filtered->is_dense = true;

    if (cloud_z_filtered->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No points after Z filtering");
      return;
    }

    // ----- Clustering using Euclidean Cluster Extraction -----
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_z_filtered);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);  // 5cm tolerance (from original)
    ec.setMinClusterSize(10);
    
    // Set a lower maximum cluster size to trim overly dense clusters
    const size_t max_cluster_size_threshold = 10000;
    ec.setMaxClusterSize(max_cluster_size_threshold);
    
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_z_filtered);
    ec.extract(cluster_indices);

    // For any cluster that exceeds the max threshold, trim it
    for (auto & indices : cluster_indices) {
      if (indices.indices.size() > max_cluster_size_threshold) {
        indices.indices.resize(max_cluster_size_threshold);
      }
    }

    int num_clusters = cluster_indices.size();
    RCLCPP_INFO(this->get_logger(), "Found %d initial clusters", num_clusters);

    // Structure to store each cluster's data
    struct ClusterData {
      Eigen::Vector4f centroid;
      std::vector<int> indices;
      float min_z;
      float max_z;
      float height;
    };
    std::vector<ClusterData> clusters(cluster_indices.size());

    // Compute centroids and heights in parallel using OpenMP
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cluster_indices.size()); ++i) {
      Eigen::Vector4f sum = Eigen::Vector4f::Zero();
      float min_z = std::numeric_limits<float>::max();
      float max_z = -std::numeric_limits<float>::max();
      
      const auto & idx = cluster_indices[i].indices;
      for (int index : idx) {
        const auto& pt = cloud_z_filtered->points[index];
        sum += pt.getVector4fMap();
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
      }
      
      Eigen::Vector4f centroid = sum / static_cast<float>(idx.size());
      float height = max_z - min_z;
      
      clusters[i] = {centroid, idx, min_z, max_z, height};
    }

    // Merge clusters whose centroids are closer than a threshold
    const float merge_threshold = 0.3; // 30cm merging threshold
    std::vector<bool> merged(clusters.size(), false);
    std::vector<ClusterData> merged_clusters;
    
    for (size_t i = 0; i < clusters.size(); ++i) {
      if (merged[i])
        continue;
        
      // Start with cluster 'i'
      ClusterData merged_cluster = clusters[i];
      merged[i] = true;
      
      // Look for clusters to merge with the current one
      for (size_t j = i + 1; j < clusters.size(); ++j) {
        if (merged[j])
          continue;
          
        // Compute Euclidean distance between centroids (x,y,z only)
        if ((clusters[i].centroid - clusters[j].centroid).head<3>().norm() < merge_threshold) {
          size_t count_i = merged_cluster.indices.size();
          size_t count_j = clusters[j].indices.size();
          
          // Recompute the weighted centroid
          merged_cluster.centroid = (merged_cluster.centroid * count_i +
                                   clusters[j].centroid * count_j) / static_cast<float>(count_i + count_j);
          
          // Update z range
          merged_cluster.min_z = std::min(merged_cluster.min_z, clusters[j].min_z);
          merged_cluster.max_z = std::max(merged_cluster.max_z, clusters[j].max_z);
          merged_cluster.height = merged_cluster.max_z - merged_cluster.min_z;
          
          // Merge indices
          merged_cluster.indices.insert(merged_cluster.indices.end(), 
                                      clusters[j].indices.begin(), 
                                      clusters[j].indices.end());
          merged[j] = true;
        }
      }
      merged_clusters.push_back(merged_cluster);
    }

    RCLCPP_INFO(this->get_logger(), "Found %d merged clusters", (int)merged_clusters.size());

    // Sort cluster heights in descending order to determine a reference height
    std::vector<float> heights;
    for (const auto& cluster : merged_clusters) {
      heights.push_back(cluster.height);
    }
    
    std::sort(heights.begin(), heights.end(), std::greater<float>());
    float reference_height = 0.0f;
    if (heights.size() >= 2) {
      reference_height = heights[1];  // second tallest
      RCLCPP_INFO(this->get_logger(), "Using second tallest height: %.3f m", reference_height);
    } else if (!heights.empty()) {
      reference_height = heights[0];
      RCLCPP_INFO(this->get_logger(), "Only one cluster found, height: %.3f m", reference_height);
    }

    // Define acceptable height range
    float lower_bound = reference_height * 0.5f;
    float upper_bound = reference_height * 1.0f;
    RCLCPP_INFO(this->get_logger(), "Height range: %.3f m to %.3f m", lower_bound, upper_bound);

    // Create MarkerArray for clusters within the acceptable height range
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    for (const auto& cluster : merged_clusters) {
      if (cluster.height >= lower_bound && cluster.height <= upper_bound) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "clusters";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = cluster.centroid[0];
        marker.pose.position.y = cluster.centroid[1];
        marker.pose.position.z = cluster.centroid[2];
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = cluster.height;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
      }
    }

    marker_pub_->publish(marker_array);
  }

  bool bypass_ground_removal_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
