/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2021 Avidbots Corp.
 * @name	dynamics_limits.cpp
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

#include "flatland_plugins/dynamics_limits.h"

namespace flatland_plugins {

void DynamicsLimits::Configure(const YAML::Node &config) {
  flatland_server::YamlReader reader(config);
  acceleration_limit_ = reader.Get<double>("acceleration_limit", 0.0);  // 0.0 default disables the limit
  deceleration_limit_ = reader.Get<double>("deceleration_limit", std::numeric_limits<double>::infinity());

  // if no acceleration limit is set, default it to equal the acceleration limit
  if (deceleration_limit_ == std::numeric_limits<double>::infinity()) {
    deceleration_limit_ = acceleration_limit_;
  }

  velocity_limit_ = reader.Get<double>("velocity_limit", 0.0);   // 0.0 default disables the limit
}

double DynamicsLimits::Saturate(double in, double lower, double upper) {
  if (lower > upper) {
    return in;
  }
  double out = in;
  out = std::max(out, lower);
  out = std::min(out, upper);
  return out;
}

double DynamicsLimits::Limit(double velocity, double target_velocity, double timestep) {

  // if enabled, apply velocity limit
  if (velocity_limit_ != 0.0) {
    // Saturating the command also saturates the steering velocity
    target_velocity = DynamicsLimits::Saturate(target_velocity, -velocity_limit_, velocity_limit_);
  }

  // if the target velocity magnitude is larger, and in the same direction, we're accelerating
  double acceleration_limit;
  if (target_velocity == 0) {  // we can only be decellerating
    acceleration_limit = deceleration_limit_;
  } else if (velocity == 0) {  // we can only be accelerating
    acceleration_limit = acceleration_limit_;
  } else if (velocity*target_velocity < 0) {  // velocities are in different directions, we must be decelerating at least initially
    if (deceleration_limit_ != 0.0) {
      double initial_deceleration = DynamicsLimits::Saturate((target_velocity - velocity) / timestep, -deceleration_limit_, deceleration_limit_);
      double new_velocity = velocity + initial_deceleration * timestep;
      if (new_velocity*velocity>0 && acceleration_limit_ != 0.0)  {  // no zero velocity crossing, we're only decelerating
        acceleration_limit = deceleration_limit_;
      } else {  // We decelerate through a zero velocity crossing, both limits apply proportionally
        double deceleration_time = fabs(velocity)/deceleration_limit_;
        if (acceleration_limit_ == 0) {  // odd corner case where there's a deceleration limit but no acceleration limit
          acceleration_limit = 0;  // effectively no limit
        } else {  // we have both acceleration and deceleration limits, which apply proportionally
          acceleration_limit = deceleration_limit_ * deceleration_time/timestep + acceleration_limit_ * (1-deceleration_time/timestep);
        }
      }
      
    } else {
      velocity = 0;  // override incoming velocity, since we have no deceleration limit
      acceleration_limit = acceleration_limit_;
    }
  } else if (fabs(velocity) < fabs(target_velocity))  {  // new velocity magnitude is larger than old, and in the same direction: accelerating
    acceleration_limit = acceleration_limit_;
  } else {  // new velocity magnitude is smaller or equal than old, in the same direction: decelerating, unless there's a zero crossing inside the timestep
    acceleration_limit = deceleration_limit_;
  }

  // compute accleration, and limit it if needed
  double acceleration = (target_velocity - velocity) / timestep;
  if (acceleration_limit != 0.0) {
    acceleration =
        DynamicsLimits::Saturate(acceleration, -acceleration_limit, acceleration_limit);
  }

  // (2) Update the new steering velocity
  return velocity + acceleration * timestep;
}

}  /* end namespace flatland_plugins */