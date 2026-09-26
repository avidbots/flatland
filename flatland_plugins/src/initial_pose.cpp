#include <flatland_plugins/initial_pose.h>
#include <flatland_server/model.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <cmath>

namespace flatland_plugins
{

void InitialPose::OnInitialize(const YAML::Node & config)
{
  flatland_server::YamlReader reader(node_, config);
  initial_pose_.header.frame_id = reader.Get<std::string>("frame_id", "map");
  std::string topic = reader.Get<std::string>("topic", "/initialpose");
  odom_frame_id_ = reader.Get<std::string>("odom_frame_id", "");
  auto variance = reader.SubnodeOpt("variance", flatland_server::YamlReader::MAP);
  initial_pose_.pose.covariance[0] = variance.Get<double>("x", 0.01);
  initial_pose_.pose.covariance[7] = variance.Get<double>("y", 0.01);
  initial_pose_.pose.covariance[35] = variance.Get<double>("yaw", std::pow(M_PI / 180.0, 2));
  variance.EnsureAccessedAllKeys();
  reader.EnsureAccessedAllKeys();

  const auto * body = GetModel()->bodies_.front()->physics_body_;
  initial_pose_.pose.pose.position.x = body->GetPosition().x;
  initial_pose_.pose.pose.position.y = body->GetPosition().y;
  double yaw = body->GetAngle();
  initial_pose_.pose.pose.orientation.z = std::sin(yaw / 2.0);
  initial_pose_.pose.pose.orientation.w = std::cos(yaw / 2.0);
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic, 1);
  if (!odom_frame_id_.empty()) {
    tf_subscription_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        for (const auto & transform : message->transforms) {
          if (published_ && transform.header.frame_id == initial_pose_.header.frame_id &&
            transform.child_frame_id == odom_frame_id_)
          {
            localized_ = true;
          }
        }
      });
  }
}

void InitialPose::BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper)
{
  if (localized_ || (published_ && odom_frame_id_.empty()) ||
    publisher_->get_subscription_count() == 0) return;

  if (last_publish_time_ &&
    timekeeper.GetSimTime() - *last_publish_time_ < rclcpp::Duration::from_seconds(1.0)) return;

  initial_pose_.header.stamp = timekeeper.GetSimTime();
  publisher_->publish(initial_pose_);
  last_publish_time_ = timekeeper.GetSimTime();
  published_ = true;
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::InitialPose, flatland_server::ModelPlugin)