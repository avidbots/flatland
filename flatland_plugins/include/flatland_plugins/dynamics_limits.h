/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2021 Avidbots Corp.
 * @name	Dynamics_Limits.h
 * @brief   A generic acceleration, deceleration and velocity bounding utility class
 * @author  Joseph Duchesne
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Avidbots Corp.
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


#ifndef FLATLAND_PLUGINS_DYNAMICS_LIMITS_H
#define FLATLAND_PLUGINS_DYNAMICS_LIMITS_H

#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_plugins {

/**
 * This class implements the model plugin class and provides laser data
 * for the given configurations
 */
class DynamicsLimits {
 public:
  double acceleration_limit_ = 0.0;  // Maximum rate of change in velocity in the direction away from zero. Zero value disables this limit.
  double deceleration_limit_ = 0.0; // Maximum rate of change in velocity in the direction towards zero. Zero value disables this limit.
  double velocity_limit_ = 0.0;      // Maximum rate of change in position (in either direction). Zero value disables this limit.


  /**
   * @brief               blank constructor, no-op class
   */
  DynamicsLimits() {};

  /**
   * @name         Load configuration from a yaml object
   * @brief        Constructor from yaml configuration file node
   * @param[in]    YAML::Node& configuration node
   */
   void Configure(const YAML::Node &config);

  /**
   * @name          Saturate
   * @brief         Apply dynamics limits
   * @param[in]     config The plugin YAML node
   */
   static double Saturate(double in, double lower, double upper);

  /**
   * @name          Apply
   * @brief         Apply dynamics limits based on class configured limits
   * @param[in]     velocity The current velocity (units/second)
   * @param[in]     target_velocity The target velocity requested (units/second)
   * @param[in]     timestep  The timestep (seconds) (used to calculate acceleration from delta velocity)
   * @return        The new velocity result after target velocity has been subjected to limits
   */
   double Limit(double velocity, double target_velocity, double timestep);

};

};

#endif  // FLATLAND_PLUGINS_DYNAMICS_LIMITS_H