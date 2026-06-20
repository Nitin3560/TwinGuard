#include "twinguard_swarm_estimation_cpp/visual_odometry.hpp"

#include <algorithm>
#include <array>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace twinguard::estimation
{

class VisualOdometryNode : public rclcpp::Node
{
public:
  VisualOdometryNode()
  : Node("visual_odometry_node")
  {
    max_features_ = declare_parameter<int>("max_features", 200);
    vo_ = SparseOpticalFlowVO(max_features_);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image_raw",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        handle_image(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/depth",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        handle_depth(*msg);
      });

    vo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("visual_odometry", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "visual_odometry_diagnostics", 10);
  }

private:
  static diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  void handle_depth(const sensor_msgs::msg::Image & msg)
  {
    try {
      latest_depth_ = cv_bridge::toCvCopy(msg)->image.clone();
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "depth conversion failed: %s", ex.what());
    }
  }

  void handle_image(const sensor_msgs::msg::Image & msg)
  {
    cv::Mat gray;
    try {
      gray = cv_bridge::toCvCopy(msg, "mono8")->image.clone();
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "image conversion failed: %s", ex.what());
      return;
    }

    const auto stamp = rclcpp::Time(msg.header.stamp);
    double dt_s = 0.0;
    if (has_last_stamp_) {
      dt_s = (stamp - last_stamp_).seconds();
    }
    last_stamp_ = stamp;
    has_last_stamp_ = true;

    const auto estimate = vo_.process_frame(gray, latest_depth_, dt_s);
    if (!estimate) {
      publish_diagnostics(msg.header.stamp, "waiting_for_trackable_frame", 0.0, 0, 0.0);
      return;
    }

    geometry_msgs::msg::TwistStamped out;
    out.header = msg.header;
    out.header.frame_id = msg.header.frame_id.empty() ? "camera_link" : msg.header.frame_id;
    out.twist.linear.x = estimate->velocity_estimate[0];
    out.twist.linear.y = estimate->velocity_estimate[1];
    out.twist.linear.z = estimate->velocity_estimate[2];
    vo_pub_->publish(out);

    publish_diagnostics(
      msg.header.stamp,
      "visual_odometry_active",
      estimate->quality,
      estimate->tracked_features,
      estimate->mean_tracking_error,
      estimate->velocity_estimate);
  }

  void publish_diagnostics(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & message,
    double quality,
    int tracked_features,
    double mean_error,
    std::array<double, 3> velocity = {0.0, 0.0, 0.0})
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "twinguard/visual_odometry";
    status.hardware_id = "gazebo_camera";
    status.level = quality > 0.2 ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = message;
    status.values = {
      kv("quality", std::to_string(quality)),
      kv("tracked_features", std::to_string(tracked_features)),
      kv("mean_tracking_error", std::to_string(mean_error)),
      kv("velocity_x", std::to_string(velocity[0])),
      kv("velocity_y", std::to_string(velocity[1])),
      kv("velocity_z", std::to_string(velocity[2])),
    };

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  int max_features_{200};
  bool has_last_stamp_{false};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  cv::Mat latest_depth_;
  SparseOpticalFlowVO vo_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vo_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace twinguard::estimation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<twinguard::estimation::VisualOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
