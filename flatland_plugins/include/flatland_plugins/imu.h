/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	imu.h
 * @brief   IMU plugin
 * @author  Mike Brousseau
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

#include <Box2D/Box2D.h>
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <geometry_msgs/Twist.h>
#include <random>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#ifndef FLATLAND_PLUGINS_IMU_H
#define FLATLAND_PLUGINS_IMU_H

using namespace flatland_server;

namespace flatland_plugins {

class Imu : public flatland_server::ModelPlugin {
 public:
  ros::Publisher imu_pub_;
  ros::Publisher ground_truth_pub_;
  Body* body_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::Imu ground_truth_msg_;
  UpdateTimer update_timer_;
  bool enable_imu_pub_;  ///< YAML parameter to enable odom publishing

  std::default_random_engine rng_;
  std::array<std::normal_distribution<double>, 9> noise_gen_;
  geometry_msgs::TransformStamped imu_tf_;   ///< tf from body to IMU frame
  tf::TransformBroadcaster tf_broadcaster_;  ///< broadcast IMU frame
  std::string imu_frame_id_;
  bool broadcast_tf_;
  b2Vec2 linear_vel_local_prev;
  double pub_rate_;
  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          AfterPhysicsStep
   * @brief         override the AfterPhysicsStep method
   * @param[in]     timekeeper Tracks time in flatland
   */
  void AfterPhysicsStep(const Timekeeper& timekeeper) override;
  /**
   * @name        TwistCallback
   * @brief       callback to apply twist (velocity and omega)
   * @param[in]   timestep how much the physics time will increment
   */
  void TwistCallback(const geometry_msgs::Twist& msg);
};
};

#endif
