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

namespace flatland_server
{

ServiceManager::ServiceManager(SimulationManager * sim_man, World * world)
: world_(world), node_(world->node_), sim_man_(sim_man)
{
  using namespace std::placeholders;  // for _1, _2, ... etc
  change_rate_service_ = node_->create_service<flatland_msgs::srv::ChangeRate>(
    "change_rate", std::bind(&ServiceManager::ChangeRate, this, _1, _2, _3));
  spawn_model_service_ = node_->create_service<flatland_msgs::srv::SpawnModel>(
    "spawn_model", std::bind(&ServiceManager::SpawnModel, this, _1, _2, _3));
  delete_model_service_ = node_->create_service<flatland_msgs::srv::DeleteModel>(
    "delete_model", std::bind(&ServiceManager::DeleteModel, this, _1, _2, _3));
  move_model_service_ = node_->create_service<flatland_msgs::srv::MoveModel>(
    "move_model", std::bind(&ServiceManager::MoveModel, this, _1, _2, _3));
  pause_service_ = node_->create_service<std_srvs::srv::Empty>(
    "pause", std::bind(&ServiceManager::Pause, this, _1, _2, _3));
  resume_service_ = node_->create_service<std_srvs::srv::Empty>(
    "resume", std::bind(&ServiceManager::Resume, this, _1, _2, _3));
  toggle_pause_service_ = node_->create_service<std_srvs::srv::Empty>(
    "toggle_pause", std::bind(&ServiceManager::TogglePause, this, _1, _2, _3));

  if (spawn_model_service_) {
    RCLCPP_INFO(rclcpp::get_logger("Service Manager"), "Model spawning service ready to go");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Service Manager"), "Error starting model spawning service");
  }

  if (delete_model_service_) {
    RCLCPP_INFO(rclcpp::get_logger("Service Manager"), "Model deleting service ready to go");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Service Manager"), "Error starting model deleting service");
  }

  if (move_model_service_) {
    RCLCPP_INFO(rclcpp::get_logger("Service Manager"), "Model moving service ready to go");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Service Manager"), "Error starting model moving service");
  }
}

bool ServiceManager::ChangeRate(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<flatland_msgs::srv::ChangeRate::Request> request,
  std::shared_ptr<flatland_msgs::srv::ChangeRate::Response> response)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ServiceManager"), "Change rate requested with rate(\"%f\")", request->rate);

  try {
    sim_man_->setUpdateRate(request->rate);
    response->success = true;
    response->message = "";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::SpawnModel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<flatland_msgs::srv::SpawnModel::Request> request,
  std::shared_ptr<flatland_msgs::srv::SpawnModel::Response> response)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ServiceManager"),
    "Model spawn requested with path(\"%s\"), namespace(\"%s\"), "
    "name(\'%s\"), pose(%f,%f,%f)",
    request->yaml_path.c_str(), request->ns.c_str(), request->name.c_str(), request->pose.x,
    request->pose.y, request->pose.theta);

  Pose pose(request->pose.x, request->pose.y, request->pose.theta);

  try {
    world_->LoadModel(request->yaml_path, request->ns, request->name, pose);
    response->success = true;
    response->message = "";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string(e.what());
    RCLCPP_ERROR(
      rclcpp::get_logger("ServiceManager"), "Failed to load model! Exception: %s", e.what());
  }

  return true;
}

bool ServiceManager::DeleteModel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<flatland_msgs::srv::DeleteModel::Request> request,
  std::shared_ptr<flatland_msgs::srv::DeleteModel::Response> response)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ServiceManager"), "Model delete requested with name(\"%s\")",
    request->name.c_str());

  try {
    world_->DeleteModel(request->name);
    response->success = true;
    response->message = "";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::MoveModel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<flatland_msgs::srv::MoveModel::Request> request,
  std::shared_ptr<flatland_msgs::srv::MoveModel::Response> response)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("ServiceManager"), "Model move requested with name(\"%s\")",
    request->name.c_str());

  Pose pose(request->pose.x, request->pose.y, request->pose.theta);

  try {
    world_->MoveModel(request->name, pose);
    response->success = true;
    response->message = "";
  } catch (const std::exception & e) {
    response->success = false;
    response->message = std::string(e.what());
  }

  return true;
}

bool ServiceManager::Pause(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  world_->Pause();
  return true;
}

bool ServiceManager::Resume(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  world_->Resume();
  return true;
}

bool ServiceManager::TogglePause(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  world_->TogglePaused();
  return true;
}
};  // namespace flatland_server
