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

#include <flatland_server/debug_visualization.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <flatland_server/service_manager.h>
#include <flatland_server/world.h>
#include <ros/ros.h>

#include <exception>
#include <limits>
#include <string>

namespace flatland_server {

SimulationManager::SimulationManager(std::string world_yaml_file,
                                     double update_rate, double step_size,
                                     bool show_viz, double viz_pub_rate,
                                     bool train_mode)
    : world_(nullptr),
      update_rate_(update_rate),
      step_size_(step_size),
      show_viz_(show_viz),
      viz_pub_rate_(viz_pub_rate),
      world_yaml_file_(world_yaml_file),
      train_mode_(train_mode) {
  ROS_INFO_NAMED("SimMan",
                 "Simulation params: world_yaml_file(%s) update_rate(%f), "
                 "step_size(%f) show_viz(%s), viz_pub_rate(%f)",
                 world_yaml_file_.c_str(), update_rate_, step_size_,
                 show_viz_ ? "true" : "false", viz_pub_rate_);
  timekeeper.SetMaxStepSize(step_size_);
}

void SimulationManager::Main() {
  ROS_INFO_NAMED("SimMan", "Initializing...");
  run_simulator_ = true;
  // In training mode, our task generator need to load the map from
  // the map server, but as we know the map server will to push
  // the map topic at simulation time 1s.
  // so we make flatland update the world multiple steps to send out /clock
  // signal.
  int pre_run_steps = fmin(5 / step_size_, 1000);

  ros::WallRate rate(update_rate_);

  try {
    world_ = World::MakeWorld(world_yaml_file_);
    ROS_INFO_NAMED("SimMan", "World loaded");
  } catch (const std::exception &e) {
    ROS_FATAL_NAMED("SimMan", "%s", e.what());
    return;
  }

  if (show_viz_) world_->DebugVisualize();

  int iterations = 0;
  double filtered_cycle_util = 0;
  double min_cycle_util = std::numeric_limits<double>::infinity();
  double max_cycle_util = 0;
  double viz_update_period = 1.0f / viz_pub_rate_;
  ServiceManager service_manager(this, world_);
  // Timekeeper timekeeper;

  ROS_INFO_NAMED("SimMan", "Simulation loop started");

  // advertise: step world service server
  if (train_mode_) {
    ros::NodeHandle nh;
    step_world_service_ = nh.advertiseService(
        "step_world", &SimulationManager::callback_StepWorld, this);
  }

  while (ros::ok() && run_simulator_) {
    // for updating visualization at a given rate
    // see flatland_plugins/update_timer.cpp for this formula
    double f = 0.0;
    try {
      f = fmod(ros::WallTime::now().toSec() +
                   (rate.expectedCycleTime().toSec() / 2.0),
               viz_update_period);
    } catch (std::runtime_error &ex) {
      ROS_ERROR("Flatland runtime error: [%s]", ex.what());
    }
    bool update_viz = ((f >= 0.0) && (f < rate.expectedCycleTime().toSec()));

    if (train_mode_ == false || pre_run_steps > 0) {
      world_->Update(timekeeper);  // Step physics by ros cycle time
      pre_run_steps = fmax(--pre_run_steps, 0);
    }

    if (show_viz_ && update_viz) {
      world_->DebugVisualize(false);  // no need to update layer
      DebugVisualization::Get().Publish(
          timekeeper);  // publish debug visualization
    }

    ros::spinOnce();
    rate.sleep();

    // iterations++;
    // if (iterations > 100) {
    //   //
    //   iterations--;
    //   double cycle_time = rate.cycleTime().toSec() * 1000;
    //   double expected_cycle_time = rate.expectedCycleTime().toSec() * 1000;
    //   double cycle_util = cycle_time / expected_cycle_time * 100;  // in percent
    //   double factor = timekeeper.GetStepSize() * 1000 / expected_cycle_time;
    //   min_cycle_util = std::min(cycle_util, min_cycle_util);
    //   max_cycle_util = std::max(cycle_util, max_cycle_util);
    //   filtered_cycle_util = 0.99 * filtered_cycle_util + 0.01 * cycle_util;
    //   ROS_INFO_THROTTLE_NAMED(
    //       5, "SimMan",
    //       "utilization: min %.1f%% max %.1f%% ave %.1f%%  factor: %.1f",
    //       min_cycle_util, max_cycle_util, filtered_cycle_util, factor);
    //   if(iterations>200){
    //     iterations--;
    //     ROS_INFO_COND_NAMED(
    //         train_mode_, "SimMan",
    //         "Suggested number of environments for training: %4d",
    //         static_cast<int8>(1 / filtered_cycle_util));
    //   }
    // }
  }
  ROS_INFO_NAMED("SimMan", "Simulation loop ended");

  delete world_;
}

void SimulationManager::Shutdown() {
  ROS_INFO_NAMED("SimMan", "Shutdown called");
  run_simulator_ = false;
}

bool SimulationManager::callback_StepWorld(
    flatland_msgs::StepWorld::Request &request,
    flatland_msgs::StepWorld::Response &response) {
  try {
    // ros::WallRate rate_set(update_rate_);
    // double required_duration=request.step_time.data;
    // int num_of_steps;
    // num_of_steps=(int)(required_duration/step_size_);
    // for(int i=0;i<num_of_steps;i++){
    //   world_->Update(timekeeper);  // Step physics by ros cycle time
    //   ros::spinOnce();
    //   rate_set.sleep();
    // }
    int required_steps;
    float t = request.required_time;
    if(request == flatland_msgs::StepWorldRequest{})
    {
      required_steps = 1;
    }else 
    {
      required_steps = ceil(t/step_size_);
    };
    for (int i = 0; i < required_steps; i++)
    {
      world_->Update(timekeeper);  // Step physics by ros cycle time
    };
    last_update_time_ = ros::WallTime::now().toSec();
    response.success = true;
    std::string current_time = std::to_string(timekeeper.GetSimTime().toSec());
    response.message = "current sim time(s):  " + current_time;
    // ros::Time current_time=timekeeper.GetSimTime();

  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
    ROS_ERROR_NAMED("Service", "Failed to step world! Exception: %s", e.what());
  }
  return true;
}
};  // namespace flatland_server
