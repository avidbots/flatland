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

#include <flatland_msgs/DeleteModel.h>
#include <flatland_msgs/MoveModel.h>
#include <flatland_msgs/SpawnModel.h>
#include <flatland_server/simulation_manager.h>
#include <flatland_server/world.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#ifndef FLATLAND_PLUGIN_SERVICE_MANAGER_H
#define FLATLAND_PLUGIN_SERVICE_MANAGER_H

namespace flatland_server {

class SimulationManager;

/**
 * This class contains a collection of ROS services that the user may use
 * to work with the simulation
 */
class ServiceManager {
 public:
  World *world_;                ///< aaa handle to the simulation world
  SimulationManager *sim_man_;  ///< a handle to the simulation manager

  ros::ServiceServer spawn_model_service_;   ///< service for spawning models
  ros::ServiceServer delete_model_service_;  ///< service for deleting models
  ros::ServiceServer move_model_service_;    ///< service for moving models
  ros::ServiceServer pause_service_;   ///< service for pausing the simulation
  ros::ServiceServer resume_service_;  ///< service for resuming the simulation
  ros::ServiceServer toggle_pause_service_;  ///< service for toggling the
                                             /// pause state of the simulation

  /**
   * @brief Service manager constructor
   * @param[in] sim_man A handle to the simulation manager
   * @param[in] world A handle to the simulation world
   */
  ServiceManager(SimulationManager *sim_man, World *world);

  /**
   * @brief Callback for the spawn model service
   * @param[in] request Contains the request data for the service
   * @param[in/out] response Contains the response for the service
   */
  bool SpawnModel(flatland_msgs::SpawnModel::Request &request,
                  flatland_msgs::SpawnModel::Response &response);

  /**
   * @brief Callback for the delete model service
   * @param[in] request Contains the request data for the service
   * @param[in/out] response Contains the response for the service
   */
  bool DeleteModel(flatland_msgs::DeleteModel::Request &request,
                   flatland_msgs::DeleteModel::Response &response);

  /**
   * @brief Callback for the move model service
   * @param[in] request Contains the request data for the service
   * @param[in/out] response Contains the response for the service
   */
  bool MoveModel(flatland_msgs::MoveModel::Request &request,
                 flatland_msgs::MoveModel::Response &response);

  /**
   * @brief Callback for the pause service
   */
  bool Pause(std_srvs::Empty::Request &request,
             std_srvs::Empty::Response &response);

  /**
   * @brief Callback for the resume service
   */
  bool Resume(std_srvs::Empty::Request &request,
              std_srvs::Empty::Response &response);

  /**
   * @brief Callback for the pause toggle service
   */
  bool TogglePause(std_srvs::Empty::Request &request,
                   std_srvs::Empty::Response &response);
};
};
#endif
