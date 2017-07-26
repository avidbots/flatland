/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  bumper.h
 * @brief   Bumper plugin
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
#include <ros/ros.h>

#ifndef FLATLAND_PLUGINS_BUMPER_H
#define FLATLAND_PLUGINS_BUMPER_H

using namespace flatland_server;

namespace flatland_plugins {

/**
 * This class defines a bumper plugin that is used to publish the collisions
 * states of bodies in the model
 */
class Bumper : public ModelPlugin {
 public:
  struct ContactState {
    int num_count;
    double sum_normal_impulses[2];
    double sum_tangential_impulses[2];
    b2Vec2 points[2];
    b2Vec2 normal;
    int normal_sign;

    Body *body_A;
    Body *body_B;
    Entity *entity_B;

    ContactState();
    void Reset();
  };

  std::string topic_name_;
  std::string world_frame_id_;           ///< name of the world frame id
  std::vector<Body *> excluded_bodies_;  ///< bodies to ignore
  /// whether to publish all collisions, or strictly adhere to update rate
  bool publish_all_collisions_;
  double update_rate_;  ///< rate to publish message at

  UpdateTimer update_timer_;  ///< for managing update rate
  std::map<b2Contact *, ContactState> contact_states_;
  ros::Publisher collisions_publisher_;

  /**
 * @brief Initialization for the plugin
 * @param[in] config Plugin YAML Node
 */
  void OnInitialize(const YAML::Node &config) override;

  void BeforePhysicsStep(const Timekeeper &timekeeper) override;

  /**
   * @brief Called when just before physics update
   * @param[in] timekeeper Object managing the simulation time
   */
  void AfterPhysicsStep(const Timekeeper &timekeeper) override;

  void BeginContact(b2Contact *contact) override;

  void EndContact(b2Contact *contact) override;

  void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) override;
};
};

#endif