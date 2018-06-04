/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model_spawner.h
 * @brief	 Definition for model spawner
 * @author Chunshang Li
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

#include <flatland_server/service_manager.h>
#include <flatland_server/types.h>
#include <exception>

namespace flatland_server {

ServiceManager::ServiceManager(SimulationManager *sim_man, World *world)
    : world_(world), sim_man_(sim_man) {
  ros::NodeHandle nh;

  spawn_model_service_ =
      nh.advertiseService("spawn_model", &ServiceManager::SpawnModel, this);
  delete_model_service_ =
      nh.advertiseService("delete_model", &ServiceManager::DeleteModel, this);
  move_model_service_ =
      nh.advertiseService("move_model", &ServiceManager::MoveModel, this);
  pause_service_ = nh.advertiseService("pause", &ServiceManager::Pause, this);
  resume_service_ =
      nh.advertiseService("resume", &ServiceManager::Resume, this);
  toggle_pause_service_ =
      nh.advertiseService("toggle_pause", &ServiceManager::TogglePause, this);

  if (spawn_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model spawning service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model spawning service");
  }

  if (delete_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model deleting service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model deleting service");
  }

  if (move_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model moving service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model moving service");
  }
}

bool ServiceManager::SpawnModel(flatland_msgs::SpawnModel::Request &request,
                                flatland_msgs::SpawnModel::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager",
                  "Model spawn requested with path(\"%s\"), namespace(\"%s\"), "
                  "name(\'%s\"), pose(%f,%f,%f)",
                  request.yaml_path.c_str(), request.ns.c_str(),
                  request.name.c_str(), request.pose.x, request.pose.y,
                  request.pose.theta);

  Pose pose(request.pose.x, request.pose.y, request.pose.theta);

  try {
    world_->LoadModel(request.yaml_path, request.ns, request.name, pose);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
    ROS_ERROR_NAMED("ServiceManager", "Failed to load model! Exception: %s",
                    e.what());
  }

  return true;
}

bool ServiceManager::DeleteModel(
    flatland_msgs::DeleteModel::Request &request,
    flatland_msgs::DeleteModel::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager", "Model delete requested with name(\"%s\")",
                  request.name.c_str());

  try {
    world_->DeleteModel(request.name);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::MoveModel(flatland_msgs::MoveModel::Request &request,
                               flatland_msgs::MoveModel::Response &response) {
  ROS_DEBUG_NAMED("ServiceManager", "Model move requested with name(\"%s\")",
                  request.name.c_str());

  Pose pose(request.pose.x, request.pose.y, request.pose.theta);

  try {
    world_->MoveModel(request.name, pose);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::Pause(std_srvs::Empty::Request &request,
                           std_srvs::Empty::Response &response) {
  world_->Pause();
  return true;
}

bool ServiceManager::Resume(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response) {
  world_->Resume();
  return true;
}

bool ServiceManager::TogglePause(std_srvs::Empty::Request &request,
                                 std_srvs::Empty::Response &response) {
  world_->TogglePaused();
  return true;
}
};
