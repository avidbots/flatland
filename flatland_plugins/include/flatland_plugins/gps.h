#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#ifndef FLATLAND_PLUGINS_GPS_H
#define FLATLAND_PLUGINS_GPS_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class simulates a GPS receiver in Flatland
 */
class Gps : public ModelPlugin {
 public:
  std::string topic_;     ///< topic name to publish the GPS fix
  std::string frame_id_;  ///< GPS frame ID
  Body *body_;            ///< body the simulated GPS antenna attaches to
  Pose origin_;           ///< GPS sensor frame w.r.t the body
  double ref_lat_rad_;  ///< latitude in radians corresponding to (0, 0) in map
                        /// frame
  double ref_lon_rad_;  ///< longitude in radians corresponding to (0, 0) in map
                        /// frame
  double ref_ecef_x_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double ref_ecef_y_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double ref_ecef_z_;   ///< ECEF coordinates of reference lat and lon at zero
                        /// altitude
  double update_rate_;  ///< GPS fix publish rate
  bool broadcast_tf_;   ///< whether to broadcast laser origin w.r.t body

  static double WGS84_A;   ///< Earth's major axis length
  static double WGS84_E2;  ///< Square of Earth's first eccentricity

  ros::Publisher fix_publisher_;             ///< GPS fix topic publisher
  tf::TransformBroadcaster tf_broadcaster_;  ///< broadcast GPS frame
  geometry_msgs::TransformStamped gps_tf_;   ///< tf from body to GPS frame
  sensor_msgs::NavSatFix gps_fix_;           ///< message for publishing output
  UpdateTimer update_timer_;                 ///< for controlling update rate

  Eigen::Matrix3f m_body_to_gps_;  ///< tf from body to GPS

  /**
   * @brief Initialization for the plugin
   * @param[in] config Plugin YAML Node
   */
  void OnInitialize(const YAML::Node &config) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);

  /**
   * @brief Method to compute ECEF coordinates of reference
   * latitude and longitude, assuming zero altitude
   */
  void ComputeReferenceEcef();

  /**
   * @brief Method that updates the current state of the GPS fix output
   * for publishing
   */
  void UpdateFix();
};
}

#endif  // FLATLAND_PLUGINS_GPS_H
