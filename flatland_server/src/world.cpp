/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.cpp
 * @brief	 Loads world file
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

#include <Box2D/Box2D.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/world.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <map>
#include <string>

namespace flatland_server {

World::World() : gravity_(0, 0) { physics_world_ = new b2World(gravity_); }

World::~World() {
  for (int i = 0; i < layers_.size(); i++) {
    delete layers_[i];
  }

  delete physics_world_;
}

void World::Update(double timestep) {
  plugin_manager_.BeforePhysicsStep(timestep);
  physics_world_->Step(timestep, 200, 100);
  plugin_manager_.AfterPhysicsStep(timestep);
}

World *World::MakeWorld(const std::string &yaml_path) {
  // parse the world YAML file
  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + yaml_path, e);
  }

  if (yaml["properties"] && yaml["properties"].IsMap()) {
    // TODO (Chunshang): parse properties
  } else {
    throw YAMLException("Missing/invalid world param \"properties\"");
  }

  World *w = new World();

  try {
    w->LoadLayers(yaml_path);
    w->LoadModels(yaml_path);
  } catch (const YAML::Exception &e) {
    delete w;
    throw YAMLException(e);
  }

  return w;
}

void World::LoadLayers(const std::string &yaml_path) {
  boost::filesystem::path path(yaml_path);

  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e);
  }

  if (!yaml["layers"] || !yaml["layers"].IsSequence()) {
    throw YAMLException("Missing/invalid world param \"layers\"");
  }

  // loop through each layer and parse the data
  for (int i = 0; i < yaml["layers"].size(); i++) {
    Layer *layer;

    if (cfr_.IsLayersFull()) {
      throw YAMLException("Max number of layers reached, max is " +
                          std::to_string(cfr_.MAX_LAYERS));
    }

    layer = Layer::MakeLayer(physics_world_, &cfr_, path.parent_path(),
                             yaml["layers"][i]);

    layers_.push_back(layer);
    ROS_INFO_NAMED("Layer", "Layer %s loaded", layer->name_.c_str());
  }
}

void World::LoadModels(const std::string &yaml_path) {
  boost::filesystem::path path(yaml_path);
  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e);
  }

  // models is optional
  if (yaml["models"] && !yaml["models"].IsSequence()) {
    throw YAMLException("Invalid world param \"layers\", must be a sequence");
  } else if (yaml["models"]) {
    for (int i = 0; i < yaml["models"].size(); i++) {
      const YAML::Node &node = yaml["models"][i];
      std::string name;
      std::array<double, 3> pose;
      boost::filesystem::path model_path;

      if (node["name"]) {
        name = node["name"].as<std::string>();
      } else {
        throw YAMLException("Missing model name in model index=" +
                            std::to_string(i));
      }

      if (node["pose"] && node["pose"].IsSequence() &&
          node["pose"].size() == 3) {
        pose[0] = node["pose"][0].as<double>();
        pose[1] = node["pose"][1].as<double>();
        pose[2] = node["pose"][2].as<double>();
      } else {
        throw YAMLException("Missing/invalid \"pose\" in " + name + " model");
      }

      if (node["model"]) {
        model_path = boost::filesystem::path(node["model"].as<std::string>());
      } else {
        throw YAMLException("Missing \"model\" in " + name + " model");
      }

      if (model_path.string().front() != '/') {
        model_path = path.parent_path() / model_path;
      }

      LoadModel(model_path.string(), name, pose);
    }
  }
}

void World::LoadModel(const std::string &model_yaml_path,
                      const std::string &name,
                      const std::array<double, 3> pose) {
  Model *m = Model::MakeModel(physics_world_, &cfr_, name, model_yaml_path);
  m->TransformAll(pose);
  models_.push_back(m);

  // load model plugins, it is okay to have no plugins
  if (m->plugins_node_ && !m->plugins_node_.IsSequence()) {
    throw YAMLException("Invalid \"plugins\" in " + name +
                        " model, not a list");
  } else if (m->plugins_node_) {
    for (const auto &plugin_node : m->plugins_node_) {
      plugin_manager_.LoadModelPlugin(m, plugin_node);
    }
  }
}

void World::DebugVisualize() {
  for (auto &layer : layers_) {
    DebugVisualization::get().Visualize(
        std::string("layer_") + layer->name_, layer->body_->physics_body_,
        layer->body_->color_[0], layer->body_->color_[1],
        layer->body_->color_[2], layer->body_->color_[3]);
  }
}

};  // namespace flatland_server
