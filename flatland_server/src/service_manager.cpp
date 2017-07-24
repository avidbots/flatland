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
#include <exception>

namespace flatland_server {

ServiceManager::ServiceManager(World *world) : world_(world) {
  ros::NodeHandle nh;

  spawn_model_service_ =
      nh.advertiseService("spawn_model", &ServiceManager::SpawnModel, this);

  if (spawn_model_service_) {
    ROS_INFO_NAMED("Service Manager", "Model spawning service ready to go");
  } else {
    ROS_ERROR_NAMED("Service Manager", "Error starting model spawning service");
  }
}

// ServiceManager::~ServiceManager() {
//   ROS_ERROR("***************Unadvertise******************");
//   spawn_model_service_.shutdown();
// }

bool ServiceManager::SpawnModel(flatland_msgs::SpawnModel::Request &request,
                                flatland_msgs::SpawnModel::Response &response) {
  ROS_INFO_NAMED(
      "ModelSpawner",
      "Model spawn requested path(\"%s\"), name(\'%s\"), pose(%f,%f,%f)",
      request.yaml_path.c_str(), request.name.c_str(), request.pose.x,
      request.pose.y, request.pose.theta);

  std::array<double, 3> pose = {request.pose.x, request.pose.y,
                                request.pose.theta};

  try {
    world_->LoadModel(request.yaml_path, request.name, pose);
    response.success = true;
    response.message = "";
  } catch (const std::exception &e) {
    response.success = false;
    response.message = std::string(e.what());
  }

  return true;
}
};
