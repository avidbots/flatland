/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	tricycle.h
 * @brief   Tricycle plugin
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
#include <nav_msgs/Odometry.h>

#ifndef FLATLAND_PLUGINS_TRICYCLE_DRIVE_H
#define FLATLAND_PLUGINS_TRICYCLE_DRIVE_H

using namespace flatland_server;
using namespace std;

namespace flatland_plugins {

class TricycleDrive : public flatland_server::ModelPlugin {
 public:
  Body* body_;
  Joint* front_wj_;       ///<  front wheel joint
  Joint* rear_left_wj_;   ///< rear left wheel joint
  Joint* rear_right_wj_;  ///< rear right wheel joint
  double axel_track_;     ///< normal distrance between the rear two wheels
  double wheelbase_;      ///< distance between the front and rear wheel
  b2Vec2 rear_center_;    ///< middle point between the two rear wheels
  bool invert_steering_angle_; ///< whether to invert steering angle

  geometry_msgs::Twist twist_msg_;
  nav_msgs::Odometry odom_msg_;
  nav_msgs::Odometry ground_truth_msg_;
  ros::Subscriber twist_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher ground_truth_pub_;

  UpdateTimer update_timer_;

  default_random_engine rng_;
  array<normal_distribution<double>, 6> noise_gen_;

  /**
   * @name                OnInitialize
   * @brief               initialize the bicycle plugin
   * @param world_file    The path to the world.yaml file
   */
  void OnInitialize(const YAML::Node& config) override;

  void ComputeJoints();

  /**
  * @name          BeforePhysicsStep
  * @brief         override the BeforePhysicsStep method
  * @param[in]     config The plugin YAML node
  */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;

  /**
  * @name          TwistCallback
  * @brief         callback to apply twist (velocity and omega)
  * @param[in]     timestep how much the physics time will increment
  */
  void TwistCallback(const geometry_msgs::Twist& msg);
};
}

#endif