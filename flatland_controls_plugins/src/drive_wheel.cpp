#include <flatland_controls_plugins/drive_wheel.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace flatland_controls_plugins
{

void DriveWheel::OnInitialize(const YAML::Node & config)
{
  flatland_server::YamlReader reader(node_, config);
  const auto body_name = reader.Get<std::string>("body");
  const auto offset = reader.GetList<double>("offset", {0.0, 0.0, 0.0}, 3, 3);
  offset_ = flatland::b2Vec2(offset[0], offset[1]);
  theta_ = offset[2];
  radius_ = reader.Get<double>("radius");
  friction_ = reader.Get<double>("friction", 1.0);
  lateral_resistance_ = reader.Get<double>("lateral_resistance", 1.0);
  mode_ = reader.Get<std::string>("mode", "velocity");
  joint_name_ = reader.Get<std::string>("joint_name", GetName());
  const auto commands_topic = reader.Get<std::string>("joint_commands_topic", "/robot_joint_commands");
  const auto states_topic = reader.Get<std::string>("joint_states_topic", "/robot_joint_states");
  reader.EnsureAccessedAllKeys();

  if (radius_ <= 0.0 || friction_ < 0.0 || lateral_resistance_ < 0.0 ||
    (mode_ != "velocity" && mode_ != "effort")) {
    throw flatland_server::YAMLException("Invalid DriveWheel radius, friction or mode");
  }
  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw flatland_server::YAMLException("DriveWheel body " + body_name + " does not exist");
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

std::vector<flatland::b2Vec2> DriveWheel::GetContactPoints() const
{
  return {body_->physics_body_->GetWorldPoint(offset_)};
}

void DriveWheel::UpdateGroundContactForces(const std::vector<double> & forces)
{
  load_ = forces.empty() ? 0.0 : forces[0];
}

void DriveWheel::BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper)
{
  const double dt = timekeeper.GetStepSize();
  if (dt <= 0.0) {
    return;
  }
  auto * physics_body = body_->physics_body_;
  const auto velocity = physics_body->GetLinearVelocityFromLocalPoint(offset_);
  const auto forward = physics_body->GetWorldVector(
    flatland::b2Vec2(std::cos(theta_), std::sin(theta_)));
  const flatland::b2Vec2 lateral(-forward.y, forward.x);
  const double lateral_speed = velocity.x * lateral.x + velocity.y * lateral.y;
  const double forward_speed = velocity.x * forward.x + velocity.y * forward.y;
  const double traction = friction_ * load_;
  const double lateral_force = std::clamp(
    -lateral_resistance_ * (load_ / 9.81) * lateral_speed / dt, -traction, traction);
  double drive_force = 0.0;
  if (has_command_) {
    if (mode_ == "effort") {
      drive_force = command_ / radius_;
    } else {
      drive_force = (command_ * radius_ - forward_speed) * (load_ / 9.81) / dt;
    }
  }
  drive_force = std::clamp(
    drive_force, -std::sqrt(std::max(0.0, traction * traction - lateral_force * lateral_force)),
    std::sqrt(std::max(0.0, traction * traction - lateral_force * lateral_force)));
  const auto point = physics_body->GetWorldPoint(offset_);
  physics_body->ApplyForce(forward * drive_force + lateral * lateral_force, point);
  effort_ = drive_force * radius_;
}

void DriveWheel::AfterPhysicsStep(const flatland_server::Timekeeper & timekeeper)
{
  const auto * physics_body = body_->physics_body_;
  const auto velocity = physics_body->GetLinearVelocityFromLocalPoint(offset_);
  const auto forward = physics_body->GetWorldVector(
    flatland::b2Vec2(std::cos(theta_), std::sin(theta_)));
  const double speed = (velocity.x * forward.x + velocity.y * forward.y) / radius_;
  position_ += speed * timekeeper.GetStepSize();
  sensor_msgs::msg::JointState state;
  state.header.stamp = timekeeper.GetSimTime();
  state.name = {joint_name_};
  state.position = {position_};
  state.velocity = {speed};
  state.effort = {effort_};
  state_pub_->publish(state);
}

}  // namespace flatland_controls_plugins

PLUGINLIB_EXPORT_CLASS(flatland_controls_plugins::DriveWheel, flatland_server::ModelPlugin)