#include <arena_plugins/random_move.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>

#include <random>

namespace flatland_plugins {
using namespace std;
void RandomMove::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  string body_name = reader.Get<string>("body");
  linear_velocity_ = reader.Get<float>("linear_velocity");
  angular_velocity_max_ = reader.Get<float>("angular_velocity_max");
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with the name" + Q(body_name) + "does not exits");
  }
  // Make sure there are not unusued keys
  reader.EnsureAccessedAllKeys();
}
void RandomMove::BeforePhysicsStep(const Timekeeper& timekeeper) {
  auto v_linear = body_->physics_body_->GetLinearVelocity();
  float v_angular = 0.f;
  auto angle = body_->physics_body_->GetAngle();
  float sign = 1.0f;
  // the body collide with other objects, then inverse the linear velocity
  if (v_linear.Length() < 0.3*linear_velocity_) {
    sign = -1;
    v_angular = randomRange(-angular_velocity_max_, angular_velocity_max_);
    float v_x = sign * linear_velocity_ * cos(angle);
    float v_y = sign * linear_velocity_ * sin(angle);
    body_->physics_body_->SetLinearVelocity(b2Vec2(v_x, v_y));
  }
  body_->physics_body_->SetAngularVelocity(v_angular);
}

float RandomMove::randomRange(const float range_lo, const float range_hi) {
  static random_device r{};
  static default_random_engine e1{r()};

  uniform_real_distribution<float> nd(range_lo, range_hi);
  return nd(e1);
}

}  // namespace flatland_plugins

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomMove,
                       flatland_server::ModelPlugin)
