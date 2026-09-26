#ifndef FLATLAND_PLUGINS_INITIAL_POSE_H
#define FLATLAND_PLUGINS_INITIAL_POSE_H

#include <flatland_server/model_plugin.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <optional>

namespace flatland_plugins
{

class InitialPose : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper) override;

private:
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  std::optional<rclcpp::Time> last_publish_time_;
  std::string odom_frame_id_;
  bool published_ = false;
  bool localized_ = false;
};

}  // namespace flatland_plugins

#endif