#include <cmath>
#include <optional>

#include <Eigen/Dense>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{
double wrapAngle(double angle)
{
  while (angle > M_PI) {angle -= 2.0 * M_PI;}
  while (angle < -M_PI) {angle += 2.0 * M_PI;}
  return angle;
}
}  // namespace

// Fuses gyro-integrated heading (fast, smooth, but drifts slowly on its
// own with no absolute reference) with ICP scan-matching odometry (slower,
// occasionally has a bad single-frame estimate, but doesn't drift) using a
// standard predict/update EKF over a 3-state [x, y, yaw] pose. Same
// predict-with-one-sensor / correct-with-another pattern as
// friction-aware-planner's EKF, applied to a different quantity.
class EkfFusionNode : public rclcpp::Node
{
public:
  EkfFusionNode()
  : Node("ekf_fusion_node"), state_(Eigen::Vector3d::Zero()),
    covariance_(Eigen::Matrix3d::Identity() * 0.01)
  {
    process_noise_pos_ = declare_parameter("process_noise_pos", 0.005);
    process_noise_yaw_ = declare_parameter("process_noise_yaw", 0.01);
    measurement_noise_pos_ = declare_parameter("measurement_noise_pos", 0.05);
    measurement_noise_yaw_ = declare_parameter("measurement_noise_yaw", 0.05);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("fused_odometry", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 50, std::bind(&EkfFusionNode::onImu, this, std::placeholders::_1));
    icp_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "icp_odometry", 10, std::bind(&EkfFusionNode::onIcpOdometry, this, std::placeholders::_1));
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (!last_imu_stamp_) {
      last_imu_stamp_ = stamp;
      return;
    }
    const double dt = (stamp - *last_imu_stamp_).seconds();
    last_imu_stamp_ = stamp;
    if (dt <= 0.0 || dt > 1.0) {return;}  // clock jump or first-ever message; skip

    // Predict: only yaw actually evolves under this model (gyro-integrated
    // heading); x/y carry forward unchanged between ICP corrections. The
    // Jacobian of this step w.r.t. the state is identity -- nothing here
    // depends on the *current* x, y, or yaw -- so covariance propagation
    // is just P += Q rather than the general F*P*F^T + Q.
    state_(2) = wrapAngle(state_(2) + msg->angular_velocity.z * dt);

    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q(0, 0) = process_noise_pos_ * dt;
    Q(1, 1) = process_noise_pos_ * dt;
    Q(2, 2) = process_noise_yaw_ * dt;
    covariance_ += Q;

    publishFused(msg->header.stamp);
  }

  void onIcpOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // scan_matcher_node's output is constrained to pure yaw rotation (see
    // its planar-motion projection), so yaw recovers directly from the
    // quaternion's z/w components without a full quaternion-to-Euler call.
    const auto & q = msg->pose.pose.orientation;
    const double measured_yaw = 2.0 * std::atan2(q.z, q.w);

    Eigen::Vector3d measurement(
      msg->pose.pose.position.x, msg->pose.pose.position.y, measured_yaw);

    Eigen::Vector3d innovation = measurement - state_;
    innovation(2) = wrapAngle(innovation(2));

    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = measurement_noise_pos_;
    R(1, 1) = measurement_noise_pos_;
    R(2, 2) = measurement_noise_yaw_;

    const Eigen::Matrix3d S = covariance_ + R;
    const Eigen::Matrix3d K = covariance_ * S.inverse();

    state_ += K * innovation;
    state_(2) = wrapAngle(state_(2));
    covariance_ = (Eigen::Matrix3d::Identity() - K) * covariance_;

    publishFused(msg->header.stamp);
  }

  void publishFused(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "lidar_link";
    odom.pose.pose.position.x = state_(0);
    odom.pose.pose.position.y = state_(1);
    odom.pose.pose.orientation.z = std::sin(state_(2) / 2.0);
    odom.pose.pose.orientation.w = std::cos(state_(2) / 2.0);
    odom_pub_->publish(odom);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "fused: x=%.2f y=%.2f yaw=%.2f (cov trace=%.4f)",
      state_(0), state_(1), state_(2), covariance_.trace());
  }

  Eigen::Vector3d state_;       // [x, y, yaw]
  Eigen::Matrix3d covariance_;
  double process_noise_pos_;
  double process_noise_yaw_;
  double measurement_noise_pos_;
  double measurement_noise_yaw_;
  std::optional<rclcpp::Time> last_imu_stamp_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icp_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfFusionNode>());
  rclcpp::shutdown();
  return 0;
}
