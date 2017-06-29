/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model.cpp
 * @brief	 implements flatland model
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

#include <flatland_server/model.h>
#include <flatland_server/exceptions.h>

namespace flatland_server {

Model::Model(b2World *physics_world, const std::string &name)
    : Entity(physics_world), name_(name) {}

Model::~Model() {
  for (int i = 0; i < bodies_.size(); i++) {
    delete bodies_[i];
  }
  
  for (int i = 0; i < joints_.size(); i++) {
    delete joints_[i];
  }
}

Model *Model::make_model(b2World *physics_world,
  const boost::filesystem::path &yaml_path, const YAML::Node &model_node) {
  YAML::Node yaml;
  std::string name;

  try {
    yaml = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + yaml_path.string(), e.msg,
                        e.mark);
  }

  if (yaml["name"]) {
    name = yaml["name"].as<std::string>();
  } else {
    throw YAMLException("Missing model name");
  }

  Model *m = new Model(physics_world, name);

  // it is okay to have no plugins
  if (yaml["plugins"] && !yaml["plugins"].IsSequence()) {
    throw YAMLException("Invalid \"plugins\" in " + name + " model, not a"
                        "list");
  } else if (yaml["plugins"] && !yaml["plugins"].IsSequence()) {
    m->plugins_node_ = yaml["plugins"];
  }

  m->load_bodies(yaml["bodies"]);
  m->load_joints(yaml["joints"]);
}

void Model::load_bodies(const YAML::Node &bodies_node) {

  if (!bodies_node || !bodies_node.IsSequence() || bodies_node.size() <= 0) {
    throw YAMLException("Invalid \"bodies\" in " + name_ + " model, "
                        "must a be list of bodies of at least size 1");
  } else {
    for (const auto &body_node : bodies_node) {
      ModelBody *b = ModelBody::make_body(physics_world_, this, body_node);
      bodies_.push_back(b);
    }
  }
}

void Model::load_joints(const YAML::Node &joints_node) {

  if (joints_node && !joints_node.IsSequence()) {
    // if joints exists and it is not a sequence, it is okay to have no joints
    throw YAMLException("Invalid \"joints\" in " + name_ + " model");
  } else {
    for (const auto &joint_node : joints_node) {
      Joint *j = Joint::make_joint(physics_world_, this, joint_node);
      joints_.push_back(j);
    }
  }
}

};  // namespace flatland_server
