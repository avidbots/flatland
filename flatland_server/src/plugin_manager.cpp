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
#include <flatland_server/world.h>
#include <flatland_server/world_plugin.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

PluginManager::PluginManager() {
  model_plugin_loader_ =
      new pluginlib::ClassLoader<flatland_server::ModelPlugin>(
          "flatland_server", "flatland_server::ModelPlugin");
  world_plugin_loader_ =
      new pluginlib::ClassLoader<flatland_server::WorldPlugin>(
          "flatland_server", "flatland_server::WorldPlugin");
}

PluginManager::~PluginManager() {
  for (unsigned int i = 0; i < model_plugins_.size(); i++) {
    model_plugins_[i].reset();  // deletes a shared pointer
  }
  for (unsigned int j = 0; j < world_plugins_.size(); j++) {
    world_plugins_[j].reset();  // deletes a shared pointer
  }

  delete model_plugin_loader_;
  delete world_plugin_loader_;
}

void PluginManager::BeforePhysicsStep(const Timekeeper &timekeeper_) {
  for (const auto &model_plugin : model_plugins_) {
    model_plugin->BeforePhysicsStep(timekeeper_);
  }
  for (const auto &world_plugin : world_plugins_) {
    world_plugin->BeforePhysicsStep(timekeeper_);
  }
}

void PluginManager::AfterPhysicsStep(const Timekeeper &timekeeper_) {
  for (const auto &model_plugin : model_plugins_) {
    model_plugin->AfterPhysicsStep(timekeeper_);
  }
  for (const auto &world_plugin : world_plugins_) {
    world_plugin->AfterPhysicsStep(timekeeper_);
  }
}

void PluginManager::DeleteModelPlugin(Model *model) {
  model_plugins_.erase(
      std::remove_if(model_plugins_.begin(), model_plugins_.end(),
                     [&](boost::shared_ptr<ModelPlugin> p) {
                       return p->GetModel() == model;
                     }),
      model_plugins_.end());
}

void PluginManager::LoadModelPlugin(Model *model, YamlReader &plugin_reader) {
  std::string name = plugin_reader.Get<std::string>("name");
  std::string type = plugin_reader.Get<std::string>("type");

  // ensure no plugin with the same model and name
  if (std::count_if(model_plugins_.begin(), model_plugins_.end(),
                    [&](boost::shared_ptr<ModelPlugin> i) {
                      return i->GetName() == name && i->GetModel() == model;
                    }) >= 1) {
    throw YAMLException("Invalid \"plugins\" in " + Q(model->name_) +
                        " model, plugin with name " + Q(name) +
                        " already exists");
  }

  try {
    if (!plugin_reader.Get<bool>("enabled", "true")) {
      ROS_WARN_STREAM("Plugin "
                      << Q(model->name_) << "."
                      << plugin_reader.Get<std::string>("name", "unnamed")
                      << " disabled");
      return;
    }
  } catch (...) {
    ROS_WARN_STREAM("Body " << Q(model->name_) << "."
                            << plugin_reader.Get<std::string>("name", "unnamed")
                            << " enabled because flag failed to parse: "
                            << plugin_reader.Get<std::string>("enabled"));
  }

  // remove the name, type and enabled of the YAML Node, the plugin does not
  // need to know
  // about these parameters, remove method is broken in yaml cpp 5.2, so we
  // create a new node and add everything
  YAML::Node yaml_node;
  for (const auto &k : plugin_reader.Node()) {
    if (k.first.as<std::string>() != "name" &&
        k.first.as<std::string>() != "type" &&
        k.first.as<std::string>() != "enabled") {
      yaml_node[k.first] = k.second;
    }
  }

  boost::shared_ptr<ModelPlugin> model_plugin;

  std::string msg = "Model Plugin " + Q(name) + " type " + Q(type) + " model " +
                    Q(model->name_);

  try {
    if (type.find("::") != std::string::npos) {
      model_plugin = model_plugin_loader_->createInstance(type);
    } else {
      model_plugin =
          model_plugin_loader_->createInstance("flatland_plugins::" + type);
    }
  } catch (pluginlib::PluginlibException &e) {
    throw PluginException(msg + ": " + std::string(e.what()));
  }

  try {
    model_plugin->Initialize(type, name, model, yaml_node);
  } catch (const std::exception &e) {
    throw PluginException(msg + ": " + std::string(e.what()));
  }
  model_plugins_.push_back(model_plugin);

  ROS_INFO_NAMED("PluginManager", "%s loaded", msg.c_str());
}

void PluginManager::LoadWorldPlugin(World *world, YamlReader &plugin_reader,
                                    YamlReader &world_config) {
  std::string name = plugin_reader.Get<std::string>("name");
  std::string type = plugin_reader.Get<std::string>("type");
  ROS_INFO_NAMED("PluginManager", "finished load name and type");
  // first check for duplicate plugins
  for (auto &it : world_plugins_) {
    if (it->GetName() == name && it->GetType() == type) {
      throw YAMLException("Invalid \"world plugins\" with name " + Q(name) +
                          ", type " + Q(type) + " already exists");
    }
  }

  boost::shared_ptr<WorldPlugin> world_plugin;
  std::string msg = "World Plugin " + Q(name) + " type " + Q(type);

  YAML::Node yaml_node;
  for (const auto &k : plugin_reader.Node()) {
    if (k.first.as<std::string>() != "name" &&
        k.first.as<std::string>() != "type") {
      yaml_node[k.first] = k.second;
    }
  }

  // try to create the instance
  try {
    if (type.find("::") != std::string::npos) {
      world_plugin = world_plugin_loader_->createInstance(type);
    } else {
      world_plugin =
          world_plugin_loader_->createInstance("flatland_plugins::" + type);
    }
  } catch (pluginlib::PluginlibException &e) {
    throw PluginException(msg + ": " + std::string(e.what()));
  }

  ROS_INFO_NAMED("PluginManager", "create instance finished");

  try {
    world_plugin->Initialize(world, name, type, yaml_node, world_config);
  } catch (const std::exception &e) {
    ROS_INFO_NAMED("PluginManager", "exception happened!");
    throw PluginException(msg + ": " + std::string(e.what()));
  }
  world_plugins_.push_back(world_plugin);

  ROS_INFO_NAMED("PluginManager", "%s loaded ", msg.c_str());
}

void PluginManager::BeginContact(b2Contact *contact) {
  for (auto &model_plugin : model_plugins_) {
    model_plugin->BeginContact(contact);
  }
}

void PluginManager::EndContact(b2Contact *contact) {
  for (auto &model_plugin : model_plugins_) {
    model_plugin->EndContact(contact);
  }
}

void PluginManager::PreSolve(b2Contact *contact,
                             const b2Manifold *oldManifold) {
  for (auto &model_plugin : model_plugins_) {
    model_plugin->PreSolve(contact, oldManifold);
  }
}

void PluginManager::PostSolve(b2Contact *contact,
                              const b2ContactImpulse *impulse) {
  for (auto &model_plugin : model_plugins_) {
    model_plugin->PostSolve(contact, impulse);
  }
}

};  // namespace flatland_server
