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

#include <ros/ros.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <map>
#include <boost/filesystem.hpp>
#include <Box2D/Box2D.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/world.h>

namespace flatland_server {

World::World  () : 
  gravity_(0, 0)  {
  
  physics_world_ = new b2World(gravity_);

  ROS_INFO_NAMED("World", "World constructed");
}

World::~World() {
  delete physics_world_;

  for (int i = 0; i < layers_.size(); i++) {
    delete layers_[i];
  }

  for (int i = 0; i < models_.size(); i++) {
    delete models_[i];
  }
}

World *World::make_world(std::string yaml_path) {
  
  boost::filesystem::path path(yaml_path);

  // parse the world YAML file
  YAML::Node yaml;
  try {
     yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e.msg, e.mark);
  }
  
  if (yaml["properties"] && yaml["properties"].IsMap()) {
    // TODO (Chunshang Li): parse properties
  }
  else {
    throw YAMLException("Invalid world \"properties\"");
  }

  return new World();
}

void World::load_layers(std::string yaml_path) {
  boost::filesystem::path path(yaml_path);

  YAML::Node yaml;

  try {
     yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e.msg, e.mark);
  }

  if (!yaml["layers"] || !yaml["layers"].IsMap()) {
    throw YAMLException("Invalid world \"layers\"");
  }

  // loop through each layer and parse the data
  for (YAML::const_iterator it = yaml["layers"].begin();
  it != yaml["layers"].end(); ++it) {
    std::string name = it->first.as<std::string>();

    Layer *layer = Layer::make_layer(physics_world_, path.parent_path(), name,
      it->second);

    layers_.push_back(layer);
  }
}

void World::load_models(std::string yaml_path) {
  boost::filesystem::path path(yaml_path);

  YAML::Node yaml;

  try {
     yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e.msg, e.mark);
  }

  if (!yaml["models"] || !yaml["models"].IsSequence()) {
    throw YAMLException("Invalid world \"models\"");
  }

  // loop through each layer and parse the data
  for (int i = 0; i < yaml["models"].size(); i++) {
    // TODO (Chunshang Li): add models
  }
}

}; // namespace flatland_server
