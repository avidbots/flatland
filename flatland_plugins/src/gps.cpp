#include <flatland_plugins/gps.h>
#include <pluginlib/class_list_macros.h>

using namespace flatland_server;

namespace flatland_plugins {

double Gps::WGS84_A = 6378137.0;
double Gps::WGS84_E2 = 0.0066943799831668;

void Gps::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  update_timer_.SetRate(update_rate_);
  fix_publisher_ = nh_.advertise<sensor_msgs::NavSatFix>(topic_, 1);

  double c = cos(origin_.theta);
  double s = sin(origin_.theta);
  double x = origin_.x, y = origin_.y;
  m_body_to_gps_ << c, -s, x, s, c, y, 0, 0, 1;
}

void Gps::BeforePhysicsStep(const Timekeeper &timekeeper) {
  // keep the update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // only compute and publish when the number of subscribers is not zero
  if (fix_publisher_.getNumSubscribers() > 0) {
    UpdateFix();
    gps_fix_.header.stamp = timekeeper.GetSimTime();
    fix_publisher_.publish(gps_fix_);
  }

  if (broadcast_tf_) {
    gps_tf_.header.stamp = timekeeper.GetSimTime();
    tf_broadcaster_.sendTransform(gps_tf_);
  }
}

void Gps::ComputeReferenceEcef() {
  double s_lat = sin(ref_lat_rad_);
  double c_lat = cos(ref_lat_rad_);
  double s_lon = sin(ref_lon_rad_);
  double c_lon = cos(ref_lon_rad_);

  double n = WGS84_A / sqrt(1.0 - WGS84_E2 * s_lat * s_lat);

  ref_ecef_x_ = n * c_lat * c_lon;
  ref_ecef_y_ = n * c_lat * s_lon;
  ref_ecef_z_ = n * (1.0 - WGS84_E2) * s_lat;
}

void Gps::UpdateFix() {
  const b2Transform &t = body_->GetPhysicsBody()->GetTransform();
  Eigen::Matrix3f m_world_to_body;
  m_world_to_body << t.q.c, -t.q.s, t.p.x, t.q.s, t.q.c, t.p.y, 0, 0, 1;
  Eigen::Matrix3f m_world_to_gps = m_world_to_body * m_body_to_gps_;
  b2Vec2 gps_pos(m_world_to_gps(0, 2), m_world_to_gps(1, 2));

  /* Convert simulation position into ECEF coordinates */
  double s_lat = sin(ref_lat_rad_);
  double c_lat = cos(ref_lat_rad_);
  double s_lon = sin(ref_lon_rad_);
  double c_lon = cos(ref_lon_rad_);

  double ecef_x = ref_ecef_x_ - s_lon * gps_pos.x - s_lat * c_lon * gps_pos.y;
  double ecef_y = ref_ecef_y_ + c_lon * gps_pos.x - s_lat * s_lon * gps_pos.y;
  double ecef_z = ref_ecef_z_ + c_lat * gps_pos.y;

  /* Convert ECEF to lat/lon */
  // Longitude is easy
  gps_fix_.longitude = atan2(ecef_y, ecef_x) * 180.0 * M_1_PI;

  // Iterative solution for latitude
  double r;
  double alt;
  double p = sqrt(ecef_x * ecef_x + ecef_y * ecef_y);
  double lat_rad = atan(p / ecef_z);
  for (unsigned int i = 0; i < 4; i++) {
    double s_lat = sin(lat_rad);
    r = WGS84_A / sqrt(1.0 - WGS84_E2 * s_lat * s_lat);
    alt = p / cos(lat_rad) - r;
    lat_rad = atan(ecef_z / p / (1 - WGS84_E2 * r / (r + alt)));
  }
  gps_fix_.latitude = lat_rad * 180.0 * M_1_PI;
  gps_fix_.altitude = 0.0;
}

void Gps::ParseParameters(const YAML::Node &config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");
  topic_ = reader.Get<std::string>("topic", "gps/fix");
  frame_id_ = reader.Get<std::string>("frame", GetName());
  broadcast_tf_ = reader.Get<bool>("broadcast_tf", true);
  update_rate_ = reader.Get<double>("update_rate", 10.0);
  ref_lat_rad_ = M_PI / 180.0 * reader.Get<double>("ref_lat", 0.0);
  ref_lon_rad_ = M_PI / 180.0 * reader.Get<double>("ref_lon", 0.0);
  ComputeReferenceEcef();
  origin_ = reader.GetPose("origin", Pose(0, 0, 0));

  body_ = GetModel()->GetBody(body_name);
  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  std::string parent_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(body_->GetName()));
  std::string child_frame_id =
      tf::resolve("", GetModel()->NameSpaceTF(frame_id_));

  // Set constant frame ID in GPS fix message
  gps_fix_.header.frame_id = child_frame_id;

  // Construct constant TF transform
  gps_tf_.header.frame_id = parent_frame_id;
  gps_tf_.child_frame_id = child_frame_id;
  gps_tf_.transform.translation.x = origin_.x;
  gps_tf_.transform.translation.y = origin_.y;
  gps_tf_.transform.translation.z = 0.0;
  gps_tf_.transform.rotation.x = 0.0;
  gps_tf_.transform.rotation.y = 0.0;
  gps_tf_.transform.rotation.z = sin(0.5 * origin_.theta);
  gps_tf_.transform.rotation.w = cos(0.5 * origin_.theta);
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Gps, flatland_server::ModelPlugin)
