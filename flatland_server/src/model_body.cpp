/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 model_body.cpp
 * @brief	 implements flatland model_body
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

#include <flatland_server/model_body.h>
#include <flatland_server/exceptions.h>

namespace flatland_server {

ModelBody::ModelBody(b2World *physics_world, uint8_t body_index_, 
                     const std::string &name, 
                     const std::array<double, 4> &color, 
                     const std::array<double, 3> &origin, 
                     b2BodyType body_type)
    : physics_world_(physics_world),
      name_(name),
      color_(color),
      origin_(origin) {

  b2BodyDef body_def;
  body_def.type = body_type;

  physics_body_ = physics_world_->CreateBody(&body_def);
  physics_body_->SetUserData(this);
}

ModelBody::~ModelBody() {
  physics_body_->GetWorld()->DestroyBody(physics_body_);
}

ModelBody *ModelBody::make_body(b2World *physics_world, uint8_t model_index, 
                         const boost::filesystem::path &yaml_path,
                         const YAML::Node &model_node) {
  YAML::Node yaml;
  std::string name;

  try {
    yaml = YAML::LoadFile(yaml_path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + yaml_path.string(), e.msg,
                        e.mark);
  }

  if (yaml["name"]) {
    resolution = yaml["name"].as<std:string>();
  } else {
    throw YAMLException("Invalid \"name\" in " + name + " model");
  }

  Model *m = new Model(physics_world, model_index, name);

  if (yaml["plugins"] && yaml["plugins"].IsSequence()) {
    m->plugins_node_ = yaml["plugins"]
  } else if (yaml["plugins"] && !yaml["plugins"].IsSequence()) {
    // if plugins exists and it is not a sequence, it is okay to have no plugins
    throw YAMLException("Invalid \"plugins\" in " + name + " model");
  }

  m->load_bodies(yaml["bodies"]);
  m->load_plugins(yaml["joints"]);
}

void ModelBody::load_footprints(const YAML::Node &footprints_node) {

}

};  // namespace flatland_server
