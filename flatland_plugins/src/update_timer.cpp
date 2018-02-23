/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  update_timer.cpp
 * @brief   For managing plugin update rates
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
#include <cstdint>

namespace flatland_plugins {

UpdateTimer::UpdateTimer()
    : period_(ros::Duration(0)), last_update_time_(ros::Time(0, 0)) {}

void UpdateTimer::SetRate(double rate) {
  if (rate == 0.0)
    period_ = ros::Duration(INT32_MAX, 0);  // 1000 hours is infinity right?
  else
    period_ = ros::Duration(1.0 / rate);
}

bool UpdateTimer::CheckUpdate(const flatland_server::Timekeeper &timekeeper) {
  if (fabs(period_.toSec()) < 1e-5) {
    return true;
  }

  // Naive way of keeping the update rate, will produce update rate always
  // slightly below the desired rate
  /*
  if (now - last_update_time_ > period_) {
    last_update_time_ = now;
    return true;
  }
  return false;
  */

  // Method obtained from Hector Gazebo Plugins, works well when the step size
  // is stable and close to max step size.
  // hector_gazebo/hector_gazebo_plugins/include/hector_gazebo_plugins/update_timer.h
  double step = timekeeper.GetMaxStepSize();
  double fraction =
      fmod(timekeeper.GetSimTime().toSec() + (step / 2.0), period_.toSec());

  if ((fraction >= 0.0) && (fraction < step)) {
    last_update_time_ = timekeeper.GetSimTime();
    return true;
  }

  return false;
}
};
