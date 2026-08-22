#include <chrono>
#include <vector>

#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

extern "C" int voxel_downsample(const float *, int, float, float *, int);

class PointCloudProcessorNode : public rclcpp::Node
{
public:
  PointCloudProcessorNode()
  : Node("point_cloud_processor_node")
  {
    voxel_size_ = declare_parameter("voxel_size", 0.2);
    min_range_ = declare_parameter("min_range", 1.0);

    // The first CUDA call in a process pays for lazy context init (hundreds
    // of ms) -- absorb that here so the first real point cloud isn't the
    // one that eats it.
    float warm_in[3] = {0.0f, 0.0f, 0.0f};
    float warm_out[3];
    voxel_downsample(warm_in, 1, static_cast<float>(voxel_size_), warm_out, 1);

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points_downsampled", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", 10,
      std::bind(&PointCloudProcessorNode::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Real LiDAR data has NaN points where a ray never hit anything (e.g.
    // an upward-angled beam that clears the walls into open sky) --
    // Phase 1's synthetic scan never produced these, so this only showed up
    // once a real simulated sensor was wired in.
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(cloud, cloud, valid_indices);

    // Points are already in the LiDAR's own frame, so distance from the
    // origin here IS distance from the sensor -- cheap way to drop hits on
    // the vehicle's own chassis without needing its exact geometry.
    pcl::PointCloud<pcl::PointXYZ> range_filtered;
    range_filtered.points.reserve(cloud.points.size());
    for (const auto & p : cloud.points) {
      const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z;
      if (r2 >= min_range_ * min_range_) {range_filtered.points.push_back(p);}
    }
    cloud = range_filtered;

    const int in_count = static_cast<int>(cloud.points.size());

    std::vector<float> in_xyz(in_count * 3);
    for (int i = 0; i < in_count; ++i) {
      in_xyz[i * 3 + 0] = cloud.points[i].x;
      in_xyz[i * 3 + 1] = cloud.points[i].y;
      in_xyz[i * 3 + 2] = cloud.points[i].z;
    }
    std::vector<float> out_xyz(in_count * 3);

    const auto t0 = std::chrono::steady_clock::now();
    const int out_count = voxel_downsample(
      in_xyz.data(), in_count, static_cast<float>(voxel_size_), out_xyz.data(), in_count);
    const auto t1 = std::chrono::steady_clock::now();
    const double gpu_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    pcl::PointCloud<pcl::PointXYZ> downsampled;
    downsampled.points.resize(out_count);
    for (int i = 0; i < out_count; ++i) {
      downsampled.points[i] = pcl::PointXYZ(out_xyz[i * 3 + 0], out_xyz[i * 3 + 1], out_xyz[i * 3 + 2]);
    }
    downsampled.width = out_count;
    downsampled.height = 1;
    downsampled.is_dense = true;

    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(downsampled, out_msg);
    out_msg.header = msg->header;
    publisher_->publish(out_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%d -> %d points (voxel_size=%.2f) in %.3f ms on GPU",
      in_count, out_count, voxel_size_, gpu_ms);
  }

  double voxel_size_;
  double min_range_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
