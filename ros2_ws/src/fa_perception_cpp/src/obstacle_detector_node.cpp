#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using PointT = pcl::PointXYZ;

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode()
  : Node("obstacle_detector_node")
  {
    ground_distance_threshold_ = declare_parameter("ground_distance_threshold", 0.05);
    cluster_tolerance_ = declare_parameter("cluster_tolerance", 0.5);
    min_cluster_size_ = declare_parameter("min_cluster_size", 5);
    max_obstacle_extent_ = declare_parameter("max_obstacle_extent", 3.0);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_markers", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_downsampled", 10,
      std::bind(&ObstacleDetectorNode::onCloud, this, std::placeholders::_1));
  }

private:
  // Fits the single largest plane in the cloud with RANSAC and returns the
  // points that are NOT part of it. Good enough here because the test track
  // has exactly one dominant flat surface (the ground) -- a real outdoor
  // scene with slopes/curbs would need something more robust than "biggest
  // plane wins".
  pcl::PointCloud<PointT>::Ptr removeGroundPlane(const pcl::PointCloud<PointT>::Ptr & cloud)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_distance_threshold_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::PointCloud<PointT>::Ptr non_ground(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // keep everything EXCEPT the plane
    extract.filter(*non_ground);
    return non_ground;
  }

  std::vector<pcl::PointIndices> clusterObstacles(const pcl::PointCloud<PointT>::Ptr & cloud)
  {
    std::vector<pcl::PointIndices> cluster_indices;
    if (cloud->points.empty()) {return cluster_indices;}

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(100000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    return cluster_indices;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);

    const auto non_ground = removeGroundPlane(cloud);
    const auto clusters = clusterObstacles(non_ground);

    visualization_msgs::msg::MarkerArray markers;
    int id = 0;
    for (const auto & indices : clusters) {
      PointT min_pt, max_pt;
      pcl::PointCloud<PointT> cluster_cloud;
      for (int idx : indices.indices) {cluster_cloud.points.push_back(non_ground->points[idx]);}
      pcl::getMinMax3D(cluster_cloud, min_pt, max_pt);

      // Walls survive the single-plane ground removal (they're flat, but
      // vertical, not horizontal) and get clustered as if they were
      // obstacles. Real obstacles on this track are known to be <=1.5m --
      // anything bigger than max_obstacle_extent_ is almost certainly a
      // wall segment, not something worth reporting.
      const double extent_x = max_pt.x - min_pt.x;
      const double extent_y = max_pt.y - min_pt.y;
      if (extent_x > max_obstacle_extent_ || extent_y > max_obstacle_extent_) {continue;}

      visualization_msgs::msg::Marker box;
      box.header = msg->header;
      box.ns = "obstacles";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
      box.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
      box.pose.position.z = (min_pt.z + max_pt.z) / 2.0;
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(0.05, static_cast<double>(max_pt.x - min_pt.x));
      box.scale.y = std::max(0.05, static_cast<double>(max_pt.y - min_pt.y));
      box.scale.z = std::max(0.05, static_cast<double>(max_pt.z - min_pt.z));
      box.color.r = 0.9f; box.color.g = 0.2f; box.color.b = 0.1f; box.color.a = 0.6f;
      box.lifetime = rclcpp::Duration::from_seconds(0.5);
      markers.markers.push_back(box);

      RCLCPP_INFO(
        get_logger(), "cluster %d: %zu points, center (%.2f, %.2f, %.2f), size (%.2f x %.2f x %.2f)",
        box.id, indices.indices.size(), box.pose.position.x, box.pose.position.y, box.pose.position.z,
        box.scale.x, box.scale.y, box.scale.z);
    }
    marker_pub_->publish(markers);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%zu points -> %zu non-ground -> %zu clusters",
      cloud->points.size(), non_ground->points.size(), clusters.size());
  }

  double ground_distance_threshold_;
  double cluster_tolerance_;
  int min_cluster_size_;
  double max_obstacle_extent_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
