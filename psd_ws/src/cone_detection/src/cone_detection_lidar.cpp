#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

using std::placeholders::_1;

class LidarClusteringNode : public rclcpp::Node
{
public:
  LidarClusteringNode()
  : Node("lidar_clustering_node")
  {
    // Subscriber to the /scan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarClusteringNode::scanCallback, this, _1));

    // Publisher for the cluster markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/lidar_cones", 10);

    // Parameters: you can also make these dynamic if you like
    cluster_distance_threshold_ = 0.5;  // distance to consider points in the same cluster
    min_range_ = 0.5;
    max_range_ = 15.0;
    height_ = 0.5;  // Cylinder height
    // Cylinder radius for markers - not used in clustering,
    // just for visualization in RViz.
    cylinder_diameter_ = 0.3;
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Extract valid points within [min_range_, max_range_]
    // Convert from polar (range, angle) to Cartesian (x, y)
    std::vector<std::pair<float, float>> points;
    points.reserve(msg->ranges.size());

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r = msg->ranges[i];
      if (r >= min_range_ && r <= max_range_ && std::isfinite(r))
      {
        float x = r * std::cos(angle);
        float y = r * std::sin(angle);
        points.push_back(std::make_pair(x, y));
      }
      angle += msg->angle_increment;
    }

    // Perform naive clustering
    std::vector<std::vector<std::pair<float, float>>> clusters = clusterPoints(points);

    // Build visualization markers
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(clusters.size());

    // We'll use a unique namespace and ID for each cluster
    rclcpp::Time now = this->now();

    int id = 0;
    for (auto &cluster : clusters)
    {
      // Compute centroid
      float cx = 0.0f;
      float cy = 0.0f;
      for (auto &p : cluster)
      {
        cx += p.first;
        cy += p.second;
      }
      cx /= (float)cluster.size();
      cy /= (float)cluster.size();

      // Fill Marker
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = msg->header.frame_id; // typically "laser_frame" or "base_link"
      marker.ns = "lidar_cones";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Position (centroid)
      marker.pose.position.x = cx;
      marker.pose.position.y = cy;
      marker.pose.position.z = height_ / 2.0; // so cylinder is half above and half below z=0
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      // Scale
      marker.scale.x = cylinder_diameter_;
      marker.scale.y = cylinder_diameter_;
      marker.scale.z = height_;

      // Color (example: orange)
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;

      marker.lifetime = rclcpp::Duration(0,0); // 0 => stay until overridden
      marker_array.markers.push_back(marker);
    }

    // Publish markers
    marker_pub_->publish(marker_array);
  }

  /**
   * @brief A naive clustering approach that groups points together if they are within
   * cluster_distance_threshold_ of any point in that cluster.
   */
  std::vector<std::vector<std::pair<float, float>>>
  clusterPoints(const std::vector<std::pair<float, float>> &points)
  {
    std::vector<bool> visited(points.size(), false);
    std::vector<std::vector<std::pair<float, float>>> clusters;

    for (size_t i = 0; i < points.size(); ++i)
    {
      if (visited[i]) continue;

      // start a new cluster
      std::vector<std::pair<float, float>> cluster;
      cluster.push_back(points[i]);
      visited[i] = true;

      // We'll do a simple BFS or queue-based approach
      std::vector<size_t> neighbors;
      neighbors.push_back(i);

      while (!neighbors.empty())
      {
        size_t current_idx = neighbors.back();
        neighbors.pop_back();

        // find neighbors of current_idx
        for (size_t j = 0; j < points.size(); ++j)
        {
          if (!visited[j])
          {
            float dx = points[j].first - points[current_idx].first;
            float dy = points[j].second - points[current_idx].second;
            float dist_sq = dx*dx + dy*dy;
            if (dist_sq <= cluster_distance_threshold_ * cluster_distance_threshold_)
            {
              visited[j] = true;
              neighbors.push_back(j);
              cluster.push_back(points[j]);
            }
          }
        }
      }
      clusters.push_back(cluster);
    }

    return clusters;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  float cluster_distance_threshold_; // max distance for naive clustering
  float min_range_;
  float max_range_;
  float height_;
  float cylinder_diameter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarClusteringNode>());
  rclcpp::shutdown();
  return 0;
}
