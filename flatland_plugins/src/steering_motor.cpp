#include <flatland_plugins/steering_motor.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace flatland_plugins
{

void SteeringMotor::OnInitialize(const YAML::Node & config)
{
  flatland_server::YamlReader reader(node_, config);
  const auto joint_name = reader.Get<std::string>("joint");
  mode_ = reader.Get<std::string>("mode", "position");
  max_effort_ = reader.Get<double>("max_effort", 100.0);
  max_speed_ = reader.Get<double>("max_speed", 10.0);
  position_gain_ = reader.Get<double>("position_gain", 10.0);
  const auto commands_topic = reader.Get<std::string>("joint_commands_topic", "/robot_joint_commands");
  const auto states_topic = reader.Get<std::string>("joint_states_topic", "/robot_joint_states");
  auto limit_reader = reader.SubnodeOpt("limit", flatland_server::YamlReader::MAP);
  const double no_limit = std::numeric_limits<double>::quiet_NaN();
  const double lower = limit_reader.IsNodeNull() ? no_limit : limit_reader.Get<double>("lower", no_limit);
  const double upper = limit_reader.IsNodeNull() ? no_limit : limit_reader.Get<double>("upper", no_limit);
  if (!limit_reader.IsNodeNull()) {
    limit_reader.EnsureAccessedAllKeys();
  }
  reader.EnsureAccessedAllKeys();
  if ((mode_ != "position" && mode_ != "velocity" && mode_ != "effort") ||
    max_effort_ < 0.0 || max_speed_ <= 0.0 || position_gain_ <= 0.0) {
    throw flatland_server::YAMLException("Invalid SteeringMotor mode, effort, speed or gain");
  }
  if (!(std::isnan(lower) && std::isnan(upper)) &&
    !(std::isfinite(lower) && std::isfinite(upper) &&
    lower >= -0.95 * std::numbers::pi && upper <= 0.95 * std::numbers::pi && lower < upper)) {
    throw flatland_server::YAMLException("SteeringMotor limits must both be NaN or ordered within +/-0.95*pi");
  }
  auto * joint = GetModel()->GetJoint(joint_name);
  if (!joint || joint->physics_joint_->GetType() != flatland::e_revoluteJoint) {
    throw flatland_server::YAMLException("SteeringMotor joint " + joint_name + " must be revolute");
  }
  joint_ = static_cast<flatland::b2RevoluteJoint *>(joint->physics_joint_);
  if (!std::isnan(lower)) {
    joint_->SetLimits(lower, upper);
    joint_->EnableLimit(true);
  }
  if (mode_ != "effort") {
    joint_->SetMaxMotorTorque(max_effort_);
    joint_->EnableMotor(true);
  }
  command_sub_ = node_->create_subscription<control_msgs::msg::JointCommand>(
    GetModel()->NameSpaceTopic(commands_topic) + "/" + mode_, 1,
    [this](control_msgs::msg::JointCommand::ConstSharedPtr message) {
      if (message->interface_name != mode_ || message->joint_names.size() != message->values.size()) {
        return;
      }
      for (size_t index = 0; index < message->joint_names.size(); ++index) {
        if (message->joint_names[index] == GetName() && std::isfinite(message->values[index])) {
          command_ = message->values[index];
          has_command_ = true;
        }
      }
    });
  state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    GetModel()->NameSpaceTopic(states_topic),
    rclcpp::SensorDataQoS());
}

void SteeringMotor::BeforePhysicsStep(const flatland_server::Timekeeper &)
{
  if (!has_command_) {
    return;
  }
  if (mode_ == "effort") {
    const float torque = std::clamp(command_, -max_effort_, max_effort_);
    if (joint_->GetBodyA()->GetType() == flatland::b2_dynamicBody) {
      joint_->GetBodyA()->ApplyTorque(-torque);
    }
    if (joint_->GetBodyB()->GetType() == flatland::b2_dynamicBody) {
      joint_->GetBodyB()->ApplyTorque(torque);
    }
  } else {
    const double speed = mode_ == "velocity" ? command_ :
      position_gain_ * (command_ - joint_->GetJointAngle());
    joint_->SetMotorSpeed(std::clamp(speed, -max_speed_, max_speed_));
  }
}

void SteeringMotor::AfterPhysicsStep(const flatland_server::Timekeeper & timekeeper)
{
  sensor_msgs::msg::JointState state;
  state.header.stamp = timekeeper.GetSimTime();
  state.name = {GetName()};
  state.position = {joint_->GetJointAngle()};
  state.velocity = {joint_->GetJointSpeed()};
  state.effort = {mode_ == "effort" ?
    (has_command_ ? std::clamp(command_, -max_effort_, max_effort_) : 0.0) :
    joint_->GetMotorTorque()};
  state_pub_->publish(state);
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::SteeringMotor, flatland_server::ModelPlugin)