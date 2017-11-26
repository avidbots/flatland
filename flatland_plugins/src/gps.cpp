#include <flatland_plugins/gps.h>

using namespace flatland_server;

namespace flatland_plugins {

void Gps::OnInitialize(const YAML::Node &config)
{
  ParseParameters(config);

  update_timer_.SetRate(update_rate_);
  fix_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_, 1);
}

void Gps::BeforePhysicsStep(const Timekeeper &timekeeper)
{

}

void Gps::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "gps/fix");
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  ref_lat_ = reader.Get<double>("ref_lat", 0.0);
  ref_lon_ = reader.Get<double>("ref_lon", 0.0);
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }
}

}