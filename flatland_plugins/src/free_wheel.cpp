#include <flatland_plugins/free_wheel.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>

namespace flatland_plugins
{

void FreeWheel::OnInitialize(const YAML::Node & config)
{
  flatland_server::YamlReader reader(node_, config);
  const auto body_name = reader.Get<std::string>("body");
  const auto offset = reader.GetList<double>("offset", {0.0, 0.0, 0.0}, 3, 3);
  offset_ = flatland::b2Vec2(offset[0], offset[1]);
  theta_ = offset[2];
  radius_ = reader.Get<double>("radius");
  friction_ = reader.Get<double>("friction", 1.0);
  lateral_resistance_ = reader.Get<double>("lateral_resistance", 1.0);
  const auto encoder_topic = reader.Get<std::string>("encoder_topic", "");
  reader.EnsureAccessedAllKeys();

  if (radius_ <= 0.0 || friction_ < 0.0 || lateral_resistance_ < 0.0) {
    throw flatland_server::YAMLException("Invalid FreeWheel radius, friction or lateral_resistance");
  }
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw flatland_server::YAMLException("FreeWheel body " + body_name + " does not exist");
  }
  if (!encoder_topic.empty()) {
    encoder_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      GetModel()->NameSpaceTopic(encoder_topic), 1);
  }
}

std::vector<flatland::b2Vec2> FreeWheel::GetContactPoints() const
{
  return {body_->physics_body_->GetWorldPoint(offset_)};
}

void FreeWheel::UpdateGroundContactForces(const std::vector<double> & forces)
{
  load_ = forces.empty() ? 0.0 : forces[0];
}

void FreeWheel::BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper)
{
  auto * physics_body = body_->physics_body_;
  const auto velocity = physics_body->GetLinearVelocityFromLocalPoint(offset_);
  const auto forward = physics_body->GetWorldVector(
    flatland::b2Vec2(std::cos(theta_), std::sin(theta_)));
  const flatland::b2Vec2 lateral(-forward.y, forward.x);
  const float lateral_speed = velocity.x * lateral.x + velocity.y * lateral.y;
  const double dt = timekeeper.GetStepSize();
  if (dt > 0.0 && load_ > 0.0 && lateral_resistance_ > 0.0 && lateral_speed != 0.0f) {
    const double grip = friction_ * load_;
    const double force = std::clamp(
      -lateral_resistance_ * (load_ / 9.81) * lateral_speed / dt, -grip, grip);
    physics_body->ApplyForce(lateral * force, physics_body->GetWorldPoint(offset_));
  }

  if (encoder_pub_) {
    std_msgs::msg::Float64 encoder;
    encoder.data = (velocity.x * forward.x + velocity.y * forward.y) / radius_;
    encoder_pub_->publish(encoder);
  }
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::FreeWheel, flatland_server::ModelPlugin)