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

void PluginManager::BeforePhysicsStep(double timestep) {
  for (const auto &model_plugin : model_plugins) {
    model_plugin->BeforePhysicsStep(timestep);
  }
}

void PluginManager::AfterPhysicsStep(double timestep) {
  for (const auto &model_plugin : model_plugins) {
    model_plugin->AfterPhysicsStep(timestep);
  }
}

void PluginManager::LoadModelPlugin(Model *model,
                                    const YAML::Node &plugin_node) {
  const YAML::Node &n = plugin_node;

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

  try {
    model_plugin = class_loader_->createInstance("flatland_plugins::" + type);
  } catch (pluginlib::PluginlibException &e) {
    throw PluginException("ModelPlugin", type, name, e.what());
  }

  try {
    model_plugin->Initialize(type, name, model, plugin_node);
  } catch (const std::exception &e) {
    throw PluginException(
        "ModelPlugin", type, name,
        "Error during initialization (" + std::string(e.what()) + ")");
  }
  model_plugins.push_back(model_plugin);

  ROS_INFO_NAMED("PluginManager", "Model Plugin %s of type %s loaded",
                 name.c_str(), type.c_str());
}

void PluginManager::BeginContact(b2Contact *contact) {
  b2Fixture *f_A = contact->GetFixtureA();
  b2Fixture *f_B = contact->GetFixtureB();
  Body *b_A = static_cast<Body *>(f_A->GetBody()->GetUserData());
  Body *b_B = static_cast<Body *>(f_B->GetBody()->GetUserData());
  Entity *e_A = b_A->entity_;
  Entity *e_B = b_B->entity_;

  for (auto &model_plugin : model_plugins) {
    Model *m = model_plugin->model_;

    if (e_A == m && e_B->Type() == Entity::LAYER) {
      model_plugin->BeginContactWithMap(dynamic_cast<Layer *>(e_B), f_B, f_A,
                                        contact);
    } else if (e_A == m && e_B->Type() == Entity::MODEL) {
      model_plugin->BeginContactWithModel(dynamic_cast<Model *>(e_B), f_B, f_A,
                                          contact);
    } else if (e_B == m && e_A->Type() == Entity::LAYER) {
      model_plugin->BeginContactWithMap(dynamic_cast<Layer *>(e_A), f_A, f_B,
                                        contact);
    } else if (e_B == m && e_A->Type() == Entity::MODEL) {
      model_plugin->BeginContactWithModel(dynamic_cast<Model *>(e_A), f_A, f_B,
                                          contact);
    }

    model_plugin->BeginContact(contact);
  }
}

void PluginManager::EndContact(b2Contact *contact) {
  b2Fixture *f_A = contact->GetFixtureA();
  b2Fixture *f_B = contact->GetFixtureB();
  Body *b_A = static_cast<Body *>(f_A->GetBody()->GetUserData());
  Body *b_B = static_cast<Body *>(f_B->GetBody()->GetUserData());
  Entity *e_A = b_A->entity_;
  Entity *e_B = b_B->entity_;

  for (auto &model_plugin : model_plugins) {
    Model *m = model_plugin->model_;

    if (e_A == m && e_B->Type() == Entity::LAYER) {
      model_plugin->EndContactWithMap(dynamic_cast<Layer *>(e_B), f_B, f_A,
                                      contact);
    } else if (e_A == m && e_B->Type() == Entity::MODEL) {
      model_plugin->EndContactWithModel(dynamic_cast<Model *>(e_B), f_B, f_A,
                                        contact);
    } else if (e_B == m && e_A->Type() == Entity::LAYER) {
      model_plugin->EndContactWithMap(dynamic_cast<Layer *>(e_A), f_A, f_B,
                                      contact);
    } else if (e_B == m && e_A->Type() == Entity::MODEL) {
      model_plugin->EndContactWithModel(dynamic_cast<Model *>(e_A), f_A, f_B,
                                        contact);
    }

    model_plugin->EndContact(contact);
  }
}

};  // namespace flatland_server