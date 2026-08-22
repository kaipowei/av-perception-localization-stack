#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudProcessorNode : public rclcpp::Node
{
public:
  PointCloudProcessorNode()
  : Node("point_cloud_processor_node")
  {
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", 10,
      std::bind(&PointCloudProcessorNode::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = min_x, max_y = max_x;
    float min_z = min_x, max_z = max_x;
    for (const auto & p : cloud.points) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
      min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "%zu points, bounds x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]",
      cloud.points.size(), min_x, max_x, min_y, max_y, min_z, max_z);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
