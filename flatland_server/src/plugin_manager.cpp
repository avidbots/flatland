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
#include <yaml-cpp/yaml.h>

namespace flatland_server {

PluginManager::PluginManager() {
  class_loader_ = new pluginlib::ClassLoader<flatland_server::ModelPlugin>(
      "flatland_server", "flatland_server::ModelPlugin");
}

PluginManager::~PluginManager() { delete class_loader_; }

void PluginManager::BeforePhysicsStep(const Timekeeper &timekeeper_) {
  for (const auto &model_plugin : model_plugins_) {
    model_plugin->BeforePhysicsStep(timekeeper_);
  }
}

void PluginManager::AfterPhysicsStep(const Timekeeper &timekeeper_) {
  for (const auto &model_plugin : model_plugins_) {
    model_plugin->AfterPhysicsStep(timekeeper_);
  }
}

void PluginManager::DeleteModelPlugin(Model *model) {
  model_plugins_.erase(
      std::remove_if(model_plugins_.begin(), model_plugins_.end(),
                     [&](boost::shared_ptr<ModelPlugin> p) {
                       return p->model_ == model;
                     }),
      model_plugins_.end());
  printf("hello\n");
}

void PluginManager::LoadModelPlugin(Model *model, YamlReader &plugin_reader) {
  std::string name = plugin_reader.Get<std::string>("name");
  std::string type = plugin_reader.Get<std::string>("type");

  // ensure no plugin with the same model and name
  if (std::count_if(model_plugins_.begin(), model_plugins_.end(),
                    [&](boost::shared_ptr<ModelPlugin> i) {
                      return i->name_ == name && i->model_ == model;
                    }) >= 1) {
    throw YAMLException("Invalid \"plugins\" in " + Q(model->name_) +
                        " model, plugin with name " + Q(name) +
                        " already exists");
  }

  // remove the name and type of the YAML Node, the plugin does not need to know
  // about these parameters, remove method is broken in yaml cpp 5.2, so we
  // create a new node and add everything
  YAML::Node yaml_node;
  for (const auto &k : plugin_reader.Node()) {
    if (k.first.as<std::string>() != "name" &&
        k.first.as<std::string>() != "type") {
      yaml_node[k.first] = k.second;
    }
  }

  boost::shared_ptr<ModelPlugin> model_plugin;

  std::string msg = "Model Plugin " + Q(name) + " type " + Q(type) + " model " +
                    Q(model->name_);

  try {
    model_plugin = class_loader_->createInstance("flatland_plugins::" + type);
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