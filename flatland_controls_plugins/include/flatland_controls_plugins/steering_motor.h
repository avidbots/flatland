#ifndef FLATLAND_CONTROLS_PLUGINS_STEERING_MOTOR_H
#define FLATLAND_CONTROLS_PLUGINS_STEERING_MOTOR_H

#include <flatland_server/model_plugin.h>
#include <control_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace flatland_controls_plugins
{

class SteeringMotor : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper) override;
  void AfterPhysicsStep(const flatland_server::Timekeeper & timekeeper) override;

  flatland::b2RevoluteJoint * joint_ = nullptr;
  double max_effort_ = 100.0;
  double max_speed_ = 10.0;
  double position_gain_ = 10.0;
  double command_ = 0.0;
  bool has_command_ = false;
  std::string mode_;
  rclcpp::Subscription<control_msgs::msg::JointCommand>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
};

}  // namespace flatland_controls_plugins

#endif  // FLATLAND_CONTROLS_PLUGINS_STEERING_MOTOR_H