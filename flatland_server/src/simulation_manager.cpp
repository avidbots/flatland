/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	simulation_manager.cpp
 * @brief	Simulation manager runs the physics+event loop
 * @author Joseph Duchesne
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

#include "flatland_server/simulation_manager.h"
#include <ros/ros.h>
#include <string>
#include "flatland_server/debug_visualization.h"

namespace flatland_server {

/**
 * @name  Simulation Manager constructor
 * @param world_file   The path to the world.yaml file we wish to load
 * @param initial_rate The physics step frequency in Hz
 */
SimulationManager::SimulationManager(std::string world_file, float initial_rate)
    : gravity_(0.0, 0.0), initial_rate_(initial_rate) {
  ROS_INFO_NAMED("SimMan", "Initializing");
  physics_world_ = new b2World(gravity_);
  world_ = new World(world_file, physics_world_);
  // Todo: Initialize SimTime class here once written
}

void SimulationManager::Main() {
  ROS_INFO_NAMED("SimMan", "Main starting");

  ros::Rate rate(initial_rate_);

  while (ros::ok() && run_simulator_) {
    // Step physics by ros cycle time
    physics_world_->Step(rate.expectedCycleTime().toSec(), 10, 10);

    ros::spinOnce();  // Normal ROS event loop
    // Todo: Update bodies

    DebugVisualization::get().publish();  // Publish debug visualization output

    rate.sleep();

    ROS_INFO_THROTTLE_NAMED(
        1.0, "SimMan", "cycle time %.2f/%.2fms (%.1f%%)",
        rate.cycleTime().toSec() * 1000,
        rate.expectedCycleTime().toSec() * 1000.0,
        100.0 * rate.cycleTime().toSec() / rate.expectedCycleTime().toSec());
  }

  ROS_INFO_NAMED("SimMan", "Main exiting");
}

void SimulationManager::Shutdown() {
  ROS_INFO_NAMED("SimMan", "Shutdown called");
  delete world_;
  delete physics_world_;
}

};  // namespace flatland_server
