/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  laser.h
 * @brief   Laser plugin
 * @author  Chunshang Li
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Avidbots Corp.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Avidbots Corp. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#ifndef FLATLAND_PLUGINS_LASER_H
#define FLATLAND_PLUGINS_LASER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class Laser : public ModelPlugin, public b2RayCastCallback {
 public:
  std::string topic_;             ///< topic name to publish the laser scan
  Body *body_;                    ///<  body the laser frame attaches to
  std::array<double, 3> origin_;  ///< laser frame w.r.t the body
  double range_;                  ///< laser max range
  double max_angle_;              /// < laser max angle
  double min_angle_;              ///< laser min angle
  double increment_;              ///< laser angle increment
  double update_rate_;            ///< the rate laser scan will be published
  std::string frame_id_;          ///< laser frame id name
  uint16_t layers_bits_;  ///< for setting the layers where laser will function

  Eigen::Matrix3f m_body_to_laser_;       ///< tf from body to laser
  Eigen::Matrix3f m_world_to_body_;       ///< tf  from world to body
  Eigen::Matrix3f m_world_to_laser_;      ///< tf from world to laser
  Eigen::MatrixXf m_laser_points_;        ///< laser points in the laser' frame
  Eigen::MatrixXf m_world_laser_points_;  /// laser point in the world frame
  Eigen::Vector3f v_zero_point_;          ///< point representing (0,0)
  Eigen::Vector3f v_world_laser_origin_;  ///< (0,0) in the laser frame
  sensor_msgs::LaserScan laser_scan_;     ///< for publishing laser scan
  bool did_hit_;    ///< Box2D ray trace checking if ray hits anything
  float fraction_;  ///< Box2D ray trace fraction

  ros::Publisher scan_publisher;              ///< ros laser topic publisher
  tf::TransformBroadcaster tf_broadcaster;    ///< broadcast laser frame
  geometry_msgs::TransformStamped static_tf;  ///< tf from body to laser frame
  UpdateTimer update_timer_;                  ///< for controlling update rate

  /**
   * @brief Box2D raytrace call back method required for implementing the
   * b2RayCastCallback abstract class
   * @param[in] fixture Fixture the ray hits
   * @param[in] point Point the ray hits the fixture
   * @param[in] normal Vector indicating the normal at the point hit
   * @param[in] fraction Fraction of ray length at hit point
   */
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      const b2Vec2 &normal, float fraction) override;

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
};

#endif