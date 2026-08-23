#include <cmath>

#include <Eigen/Geometry>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointT = pcl::PointXYZ;

// Frame-to-frame LiDAR odometry: ICP-align each new scan against the
// previous one to estimate how far the vehicle moved, then accumulate.
// No map, no loop closure -- this is odometry, not full SLAM; drift is
// expected to grow unbounded over time, which is exactly what the drift
// measurement (comparing against Gazebo's ground-truth pose) is for.
class ScanMatcherNode : public rclcpp::Node
{
public:
  ScanMatcherNode()
  : Node("scan_matcher_node"), global_transform_(Eigen::Matrix4f::Identity()),
    last_final_transformation_(Eigen::Matrix4f::Identity())
  {
    max_iterations_ = declare_parameter("max_iterations", 30);
    // ICP can only find a correct correspondence for a point if the
    // *true* matching point in the other scan lies within this distance
    // of where alignment currently thinks it is. Too tight and, whenever
    // a frame's actual displacement exceeds it, ICP has literally no way
    // to discover the correct match -- it reports "converged" (iteration
    // terminated) while having barely moved anything, silently wrong
    // rather than erroring out. Set well above the largest displacement
    // expected between consecutive scans.
    max_correspondence_distance_ = declare_parameter("max_correspondence_distance", 2.0);
    ground_distance_threshold_ = declare_parameter("ground_distance_threshold", 0.05);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("icp_odometry", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_downsampled", 10,
      std::bind(&ScanMatcherNode::onCloud, this, std::placeholders::_1));
  }

private:
  // Point-to-point ICP has almost no gradient constraining motion *within*
  // a matching flat plane -- two mostly-ground-plane scans can slide past
  // each other with barely any change in point-to-point distance, so the
  // optimizer has little basis for how far it actually moved and drifts
  // wildly (observed: ~29m error over one loop, including nonsensical
  // z-axis drift for a vehicle that never leaves the ground plane). Same
  // RANSAC largest-plane removal as obstacle_detector_node, applied here
  // so ICP only ever matches wall/obstacle geometry, which actually
  // constrains the alignment.
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
    extract.setNegative(true);
    extract.filter(*non_ground);
    return non_ground;
  }

  // Drops everything but x, y translation and yaw rotation from a 4x4
  // transform -- see the call site for why.
  static Eigen::Matrix4f constrainToPlanarMotion(const Eigen::Matrix4f & transform)
  {
    const float yaw = std::atan2(transform(1, 0), transform(0, 0));
    Eigen::Matrix4f planar = Eigen::Matrix4f::Identity();
    planar.block<2, 2>(0, 0) = Eigen::Rotation2Df(yaw).toRotationMatrix();
    planar(0, 3) = transform(0, 3);
    planar(1, 3) = transform(1, 3);
    return planar;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    pcl::PointCloud<PointT>::Ptr cloud = removeGroundPlane(raw_cloud);

    if (!previous_cloud_ || cloud->points.size() < 10) {
      if (cloud->points.size() >= 10) {previous_cloud_ = cloud;}
      return;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(previous_cloud_);
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    pcl::PointCloud<PointT> aligned;
    // Seed with last step's source->target transform (constant-velocity
    // assumption) instead of identity -- gets ICP's very first iteration's
    // correspondences roughly in the right place instead of starting from
    // "assume nothing moved," which is often exactly wrong.
    icp.align(aligned, last_final_transformation_);

    if (!icp.hasConverged()) {
      RCLCPP_WARN(get_logger(), "ICP did not converge on this frame pair, skipping");
      previous_cloud_ = cloud;
      return;
    }

    // getFinalTransformation() maps the current (source) scan's points
    // onto the previous (target) scan's frame: T * P_new ~= P_old. Since
    // a world-fixed point observed from the new vehicle pose sits at
    // P_new = P_world - motion, solving gives T = +motion directly --
    // no inversion needed. (First implementation inverted this on the
    // assumption T was the *opposite* of the vehicle's motion; empirically
    // wrong by a sign flip, caught by comparing a single known 2m teleport
    // against Gazebo's ground truth -- see docs/learning-log.md.)
    //
    // This vehicle only ever translates in x/y and yaws about z -- no
    // roll, pitch, or z motion is physically possible. Rather than debug
    // why point-to-point ICP occasionally finds spurious motion in those
    // directions (observed: z drifting to +20m over one loop while x/y
    // stayed within ~5m of ground truth), project the estimate onto that
    // known 3-DOF (x, y, yaw) planar motion model before using it at all
    // -- a standard simplification for ground vehicles, not a hack around
    // an unexplained bug.
    last_final_transformation_ = constrainToPlanarMotion(icp.getFinalTransformation());
    global_transform_ = global_transform_ * last_final_transformation_;

    publishOdometry(msg->header.stamp, icp.getFitnessScore());
    previous_cloud_ = cloud;
  }

  void publishOdometry(const rclcpp::Time & stamp, double fitness_score)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "lidar_link";

    odom.pose.pose.position.x = global_transform_(0, 3);
    odom.pose.pose.position.y = global_transform_(1, 3);
    odom.pose.pose.position.z = global_transform_(2, 3);

    const Eigen::Quaternionf q(global_transform_.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "icp odom: x=%.2f y=%.2f z=%.2f (fitness=%.4f)",
      odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z,
      fitness_score);
  }

  int max_iterations_;
  double max_correspondence_distance_;
  double ground_distance_threshold_;
  Eigen::Matrix4f global_transform_;
  Eigen::Matrix4f last_final_transformation_;
  pcl::PointCloud<PointT>::Ptr previous_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMatcherNode>());
  rclcpp::shutdown();
  return 0;
}
