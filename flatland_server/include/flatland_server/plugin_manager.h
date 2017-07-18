/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 plugin_manager.h
 * @brief	 Definition for plugin manager
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

#ifndef FLATLAND_PLUGIN_MANAGER_H
#define FLATLAND_PLUGIN_MANAGER_H

#include <Box2D/Box2D.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>
#include <flatland_server/time_keeper.h>

namespace flatland_server {

class PluginManager {
  pluginlib::ClassLoader<flatland_server::ModelPlugin> *class_loader_;

 public:
  std::vector<boost::shared_ptr<ModelPlugin>> model_plugins;

  /**
   * @brief Plugin manager constructor
   */
  PluginManager();

  /**
   * @brief Plugin manager destructor
   */
  ~PluginManager();

  /**
   * @brief This method is called before the Box2D physics step
   * @param[in] time_keeper provide time related information
   */
  void BeforePhysicsStep(const TimeKeeper &time_keeper);

  /**
   * @brief This method is called after the Box2D physics step
   * @param[in] time_keeper provide time related information
   */
  void AfterPhysicsStep(const TimeKeeper &time_keeper);

  /**
   * @brief Load model plugins
   * @param[in] model The model that this plugin is tied to
   * @param[in] plugin_node The YAML node with the plugin parameter
   */
  void LoadModelPlugin(Model *model, const YAML::Node &plugin_node);

  /**
   * @brief Method called for a box2D begin contact
   * @param[in] contact Box2D contact information
   */
  void BeginContact(b2Contact *contact);

  /**
   * @brief Method called for a box2D end contact
   * @param[in] contact Box2D contact information
   */
  void EndContact(b2Contact *contact);
};
};      // namespace flatland_server
#endif  // FLATLAND_PLUGIN_MANAGER_H
