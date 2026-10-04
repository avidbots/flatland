#ifndef FLATLAND_CONTROLS_PLUGINS_DRIVE_WHEEL_H
#define FLATLAND_CONTROLS_PLUGINS_DRIVE_WHEEL_H

#include <flatland_server/model_plugin.h>
#include <control_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace flatland_controls_plugins
{

class DriveWheel : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;
  bool HasContactPoints() const override { return true; }
  std::vector<flatland::b2Vec2> GetContactPoints() const override;
  void UpdateGroundContactForces(const std::vector<double> & forces) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper) override;
  void AfterPhysicsStep(const flatland_server::Timekeeper & timekeeper) override;

  flatland_server::Body * body_ = nullptr;
  flatland::b2Vec2 offset_;
  double theta_ = 0.0;
  double radius_ = 0.0;
  double friction_ = 1.0;
  double lateral_resistance_ = 1.0;
  double load_ = 0.0;
  double position_ = 0.0;
  double effort_ = 0.0;
  double command_ = 0.0;
  bool has_command_ = false;
  std::string mode_;
  std::string joint_name_;
  rclcpp::Subscription<control_msgs::msg::JointCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
};

}  // namespace flatland_controls_plugins

#endif  // FLATLAND_CONTROLS_PLUGINS_DRIVE_WHEEL_H