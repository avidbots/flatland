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
#include <flatland_server/exceptions.h>
#include <flatland_server/world.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <map>
#include <string>

namespace flatland_server {

World::World() : gravity_(0, 0) {
  physics_world_ = new b2World(gravity_);
}

World::~World() {
  for (int i = 0; i < layers_.size(); i++) {
    delete layers_[i];
  }

  delete physics_world_;
}

World *World::make_world(std::string yaml_path) {

  // parse the world YAML file
  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + yaml_path, e.msg, e.mark);
  }


  if (yaml["properties"] && yaml["properties"].IsMap()) {
    try {
      // TODO (Chunshang): parse properties
    } catch (const YAML::Exception &e) {
      throw YAMLException("Error loading world properties from" + yaml_path, 
        e.msg, e.mark);
    }
  } else {
    throw YAMLException("Missing/invalid world param \"properties\"");
  }

  World *w = new World();
  
  try {
    w->load_layers(yaml_path);
    w->load_models(yaml_path);
  } catch (const YAMLException &e) {
    delete w;
    throw e;
  }

  return w;
}

void World::load_layers(std::string yaml_path) {
  boost::filesystem::path path(yaml_path);

  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + path.string(), e.msg, e.mark);
  }

  if (!yaml["layers"] || !yaml["layers"].IsSequence()) {
    throw YAMLException("Missing/invalid world param \"layers\"");
  }

  // loop through each layer and parse the data
  for (int i = 0; i < yaml["layers"].size(); i++) {
    Layer *layer;

    if (cfr_.IsLayersFull()) {
      throw YAMLException("Max number of layers reached, max is " + 
        cfr_.MAX_LAYERS);
    }

    try {

      layer = Layer::make_layer(physics_world_, &cfr_, 
                                path.parent_path(), yaml["layers"][i]);
    } catch (const YAML::Exception &e) {
      throw YAMLException("Error loading layer from " + yaml_path, 
        e.msg, e.mark);
    }

    layers_.push_back(layer);
    ROS_INFO_NAMED("Layer", "Layer %s loaded", layer->name_.c_str());
  }
}

void World::load_model(std::string yaml_path) {
  
}

void World::load_models(std::string yaml_path) {}

};  // namespace flatland_server
