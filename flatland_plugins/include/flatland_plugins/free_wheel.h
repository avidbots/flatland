#ifndef FLATLAND_PLUGINS_FREE_WHEEL_H
#define FLATLAND_PLUGINS_FREE_WHEEL_H

#include <flatland_server/model_plugin.h>
#include <std_msgs/msg/float64.hpp>

namespace flatland_plugins
{

class FreeWheel : public flatland_server::ModelPlugin
{
public:
  void OnInitialize(const YAML::Node & config) override;
  void BeforePhysicsStep(const flatland_server::Timekeeper & timekeeper) override;

  flatland_server::Body * body_ = nullptr;
  flatland::b2Vec2 offset_;
  double theta_ = 0.0;
  double radius_ = 0.0;
  double lateral_resistance_ = 1.0;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr encoder_pub_;
};

}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_FREE_WHEEL_H