/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	Bicycle.h
 * @brief   Bicycle plugin
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
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include "geometry_msgs/Twist.h"

#ifndef FLATLAND_PLUGINS_TRICYCLE_DRIVE_H
#define FLATLAND_PLUGINS_TRICYCLE_DRIVE_H

namespace flatland_plugins {

class TricycleDrive : public flatland_server::ModelPlugin {
 public:
  ros::Subscriber sub;
  b2Body* robot;
  b2Vec2 robot_position;
  b2Fixture* front_wheel_fixture;
  double robot_angle;
  double robot_alpha;
  double time_step;
  double velocity;
  double omega;
  bool model_is_dynamic;
  double speedFactor = 1.0;

  /*
  * @Usage
  *
  *     Include the following in the model's Yaml file
  *
  *     plugins:
  *         - type: Bicycle # plugin class name
  *         name: bicycle # for registering list of plugins
  *         body: base
  *         origin: [0, 0, 0] # w.r.t body origin, where the twist drive is
  *         applied
  *
  *     The density of the chassis main fixture must be set to 100000.0
  */

  /**
   * @name                OnInitialize
   * @brief               initialize the bicycle plugin
   * @param world_file    The path to the world.yaml file
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
  * @name          RotateVertex
  * @brief         rotates a b2Vec2 point about the origin by angle radians
  * @param[in]     b2Vec2 vertex, point ie x and y
  * @param[in]     double angle, angle to rotate point by
  * @param[out]    b2Vec2 the new point with the rotation
  */
  b2Vec2 RotateVertex(b2Vec2 vertex, double angle);
  /**
  * @name          BeforePhysicsStep
  * @brief         override the BeforePhysicsStep method
  * @param[in]     config The plugin YAML node
  */
  void BeforePhysicsStep(
      const flatland_server::Timekeeper& timekeeper) override;
  /**
  * @name          TwistCallback
  * @brief         callback to apply twist (velocity and omega)
  * @param[in]     timestep how much the physics time will increment
  */
  void TwistCallback(const geometry_msgs::Twist& msg);
  /**
  * @name          ApplyVelocity
  * @brief         Apply the twist using either the kinematic or dynamic model
  * @param[in]     twist ros message (msg.linear and msg.angular)
  */
  void ApplyVelocity();
  /**
  * @name          CreateFrontWheel
  * @brief         create the front wheel fixture and shape
  */
  void CreateFrontWheel();
  /**
  * @name          DestroyFrontWheel
  * @brief         destroy the front wheel fixture and shape
  */
  void DestroyFrontWheel();
  /**
  * @name          RecreateFrontWheel
  * @brief         destroy then create the front wheel fixture and shape
  *                used to simulate steering the front wheel
  */
  void RecreateFrontWheel();
  /**
  * @brief          calculte the delta, ie the angle the vehicle must be rotated
  *                 by, to rotate about the ICC.
  * @param[out]     delta:change in omega calculated from the distance forward
  *                 robot moved for this physics step
  */
  double CalculateDelta(double distance);
};
};

#endif