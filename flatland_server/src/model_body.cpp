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

ModelBody::ModelBody(b2World *physics_world, 
  Model *model, const std::string &name, const std::array<double, 4> &color, 
  const std::array<double, 3> &origin, b2BodyType body_type)
    : Body(physics_world, model, name, color, origin, body_type) {}

ModelBody *ModelBody::make_body(b2World *physics_world, Model *model, 
  YAML::Node body_node) {
  std::string name;
  std::array<double, 4> color;
  std::array<double, 3> origin;
  b2BodyType type;

  if (body_node["name"]) {
    name = body_node["name"].as<std::string>();
  } else {
    throw YAMLException("Missing body name");
  }

  if (body_node["origin"] && body_node["origin"].IsSequence() &&
      body_node["origin"].size() == 3) {
    origin[0] = body_node["origin"][0].as<double>();
    origin[1] = body_node["origin"][1].as<double>();
    origin[2] = body_node["origin"][2].as<double>();
  } else {
    throw YAMLException("Missing/invalid \"origin\" in " + name + " body");
  }


  if (body_node["color"] && body_node["color"].IsSequence() &&
      body_node["color"].size() == 3) {
    color[0] = body_node["color"][0].as<double>();
    color[1] = body_node["color"][1].as<double>();
    color[2] = body_node["color"][2].as<double>();
    color[3] = body_node["color"][3].as<double>();
  } else {
    throw YAMLException("Missing/invalid \"color\" in " + name + " body");
  }

  if (body_node["type"]) {
    std::string type_str = body_node["type"].as<std::string>();

    if (type_str == "static") {
      type = b2_staticBody;
    } else if (type_str == "kinematic") {
      type = b2_kinematicBody;
    } else if (type_str == "dynamic") {
      type = b2_dynamicBody;
    } else {
      throw YAMLException("Invalid \"type\" in " + name + " body, must be "
                          "either static, kinematic, or dynamic");
    }
  } else {
    throw YAMLException("Missing \"type\" in " + name + " body");
  }

  ModelBody *m = new ModelBody(physics_world, model, name, color, origin, type);

  if (!body_node["footprints"] || !body_node["footprints"].IsSequence() ||
       body_node["footprints"].size() <= 0) {
    throw YAMLException("Missing/Invalid \"footprints\" in " + name + " body");
  } else {
    m->load_footprints(body_node["footprints"]);
  }

  return m;
}

void ModelBody::load_footprints(const YAML::Node &footprints_node) {

}

};  // namespace flatland_server
