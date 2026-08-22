#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace
{
struct Blob
{
  float cx, cy, cz, radius;
  int count;
};

// Stand-in for a real LiDAR frame until Phase 2 wires up a simulated sensor
// in Gazebo. Ground plane + two raised clusters gives the downstream nodes
// (ground segmentation, clustering) something structured to work against
// instead of pure noise.
pcl::PointCloud<pcl::PointXYZ>::Ptr synthesizeScan(std::mt19937 & rng)
{
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  std::uniform_real_distribution<float> ground_xy(-10.0f, 10.0f);
  std::uniform_real_distribution<float> ground_z(-0.02f, 0.02f);
  for (int i = 0; i < 8000; ++i) {
    cloud->points.emplace_back(ground_xy(rng), ground_xy(rng), ground_z(rng));
  }

  const std::array<Blob, 2> blobs = {
    Blob{3.0f, 2.0f, 0.5f, 0.5f, 400},
    Blob{-4.0f, -3.0f, 0.75f, 0.4f, 300},
  };
  std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
  for (const auto & blob : blobs) {
    for (int i = 0; i < blob.count; ++i) {
      cloud->points.emplace_back(
        blob.cx + unit(rng) * blob.radius,
        blob.cy + unit(rng) * blob.radius,
        std::max(0.0f, blob.cz + unit(rng) * blob.radius));
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}
}  // namespace

class PointCloudSourceNode : public rclcpp::Node
{
public:
  PointCloudSourceNode()
  : Node("point_cloud_source_node"), rng_(std::random_device{}())
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_raw", 10);
    timer_ = create_wall_timer(100ms, std::bind(&PointCloudSourceNode::publishScan, this));
    RCLCPP_INFO(get_logger(), "publishing synthetic scans on 'points_raw' at 10 Hz");
  }

private:
  void publishScan()
  {
    auto cloud = synthesizeScan(rng_);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = now();
    msg.header.frame_id = "lidar_link";
    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSourceNode>());
  rclcpp::shutdown();
  return 0;
}
