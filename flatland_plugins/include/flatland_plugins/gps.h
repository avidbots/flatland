#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

#ifndef FLATLAND_PLUGINS_GPS_H
#define FLATLAND_PLUGINS_GPS_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class simulates a GPS receiver in Flatland
 */
class Gps : public ModelPlugin {
public:
  std::string topic_;   ///< topic name to publish the GPS fix
  Body *body_;          ///< body the simulated GPS antenna attaches to
  Pose origin_;         ///< GPS sensor frame w.r.t the body
  double ref_lat_;      ///< latitude corresponding to (0, 0) in map frame
  double ref_lon_;      ///< longitude corresponding to (0, 0) in map frame
  double update_rate_;  ///< GPS fix publish rate
  bool broadcast_tf_;   ///< whether to broadcast laser origin w.r.t body

  ros::Publisher fix_publisher_;  ///< GPS fix topic publisher
  tf::TransformBroadcaster tf_broadcaster_;   ///< broadcast GPS frame
  geometry_msgs::TransformStamped gps_tf_;    ///< tf from body to GPS frame
  UpdateTimer update_timer_;                  ///< for controlling update rate

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
   * @brief helper function to extract the paramters from the YAML Node
   * @param[in] config Plugin YAML Node
   */
  void ParseParameters(const YAML::Node &config);
};

}


#endif // FLATLAND_PLUGINS_GPS_H