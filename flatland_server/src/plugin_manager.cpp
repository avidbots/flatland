/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 plugin_manager.cpp
 * @brief	 Implementation for plugin manager
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

#include <flatland_server/exceptions.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/plugin_manager.h>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

void PluginManager::BeforePhysicsStep(double timestep) {
  for (const auto &model_plugin : model_plugins) {
    model_plugin->BeforePhysicsStep(timestep);
  }
}

void PluginManager::AfterPhysicsStep(double timestep) {
  for (const auto &model_plugin : model_plugins) {
    model_plugin->BeforePhysicsStep(timestep);
  }
}

void PluginManager::LoadModelPlugin(Model *model,
                                    const YAML::Node &plugins_node) {
  const YAML::Node &n = plugins_node;

  std::string name;
  std::string type;

  if (n["name"]) {
    name = n["name"].as<std::string>();
  } else {
    throw YAMLException("Missing plugin name");
  }

  if (n["type"]) {
    type = n["type"].as<std::string>();
  } else {
    throw YAMLException("Missing \"type\" in plugin " + name);
  }

  boost::shared_ptr<ModelPlugin> model_plugin;
  pluginlib::ClassLoader<flatland_server::ModelPlugin> loader(
      "flatland_server", "flatland_server::ModelPlugin");

  try {
    model_plugin = loader.createInstance("flatland_plugins::" + type);
  } catch (pluginlib::PluginlibException &e) {
    ROS_ERROR("Failed to load %s plugin of type %s (%s)", name.c_str(),
              type.c_str(), e.what());
    return;  // TODO (Chunshang): handle this exception properly
  }

  model_plugin->Initialize(name, model, plugins_node);
  model_plugins.push_back(model_plugin);
}

};  // namespace flatland_server