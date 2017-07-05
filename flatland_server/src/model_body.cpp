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

#include <flatland_server/exceptions.h>
#include <flatland_server/model_body.h>

namespace flatland_server {

ModelBody::ModelBody(b2World *physics_world, CollisionFilterRegistry *cfr,
                     Model *model, const std::string &name,
                     const std::array<double, 4> &color,
                     const std::array<double, 3> &origin, b2BodyType body_type)
    : Body(physics_world, model, name, color, origin, body_type), cfr_(cfr) {}

ModelBody *ModelBody::MakeBody(b2World *physics_world,
                               CollisionFilterRegistry *cfr, Model *model,
                               const YAML::Node &body_node) {
  std::string name;
  std::array<double, 4> color;
  std::array<double, 3> origin;
  b2BodyType type;

  if (body_node["name"]) {
    name = body_node["name"].as<std::string>();
  } else {
    throw YAMLException("Missing a body name");
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
      body_node["color"].size() == 4) {
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
      throw YAMLException("Invalid \"type\" in " + name +
                          " body, supported body types are: "
                          "static, kinematic, dynamic");
    }
  } else {
    throw YAMLException("Missing \"type\" in " + name + " body");
  }

  ModelBody *m =
      new ModelBody(physics_world, cfr, model, name, color, origin, type);

  try {
    m->LoadFootprints(body_node["footprints"]);
  } catch (const YAML::Exception &e) {
    delete m;
    throw YAMLException(e);
  }

  return m;
}

void ModelBody::LoadFootprints(const YAML::Node &footprints_node) {
  const YAML::Node &node = footprints_node;

  bool is_node = node;

  if (!node || !node.IsSequence() || node.size() <= 0) {
    throw YAMLException("Missing/Invalid \"footprints\" in " + name_ + " body");
  } else {
    for (int i = 0; i < node.size(); i++) {
      if (node[i]["type"]) {
        std::string type = node[i]["type"].as<std::string>();

        if (type == "circle") {
          LoadCircleFootprint(node[i]);
        } else if (type == "polygon") {
          LoadPolygonFootprint(node[i]);
        } else {
          throw YAMLException("Invalid footprint \"type\" in " + name_ +
                              " body, support footprints are: circle, polygon");
        }
      } else {
        throw YAMLException("Missing footprint \"type\" in " + name_ + " body");
      }
    }
  }
}

void ModelBody::ConfigFootprintCollision(const YAML::Node &footprint_node,
                                         b2FixtureDef &fixture_def) {
  const YAML::Node &n = footprint_node;

  std::vector<std::string> layers;
  bool is_sensor = false;
  bool self_collide = false;

  if (n["is_sensor"]) {
    is_sensor = n["is_sensor"].as<bool>();
  }

  if (n["self_collide"]) {
    self_collide = n["self_collide"].as<bool>();
  }

  if (n["layers"] && !n["layers"].IsSequence()) {
    throw YAMLException("Invalid footprint \"layer\" in " + name_ +
                        " body, must be a sequence");
  } else if (n["layers"] && n["layers"].IsSequence()) {
    for (int i = 0; i < n["layers"].size(); i++) {
      std::string layer_name = n["layers"][i].as<std::string>();
      layers.push_back(layer_name);
    }
  } else {
    layers = {"all"};
  }

  if (layers.size() == 1 && layers[0] == "all") {
    layers.clear();
    cfr_->ListAllLayers(layers);
  }

  if (is_sensor) {
    fixture_def.isSensor = true;
  }

  if (self_collide) {
    fixture_def.filter.groupIndex = cfr_->RegisterCollide();
  } else {
    fixture_def.filter.groupIndex =
        (dynamic_cast<Model *>(entity_))->no_collide_group_index_;
  }

  for (const auto &layer : layers) {
    int layer_id = cfr_->LookUpLayerId(layer);

    if (layer_id < 0) {
      throw YAMLException("Invalid footprint \"layer\" in " + name_ +
                          " body, " + layer + " does not exist");
    } else {
      fixture_def.filter.categoryBits |= 1 << layer_id;
    }
  }

  fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
}

void ModelBody::LoadCircleFootprint(const YAML::Node &footprint_node) {
  const YAML::Node &n = footprint_node;

  double x, y, radius;

  if (n["center"] && n["center"].IsSequence() && n["center"].size() == 2) {
    x = n["center"][0].as<double>();
    y = n["center"][1].as<double>();
  } else {
    throw YAMLException("Missing/invalid circle footprint \"center\" in " +
                        name_ + " body");
  }

  if (n["radius"]) {
    radius = n["radius"].as<double>();
  } else {
    throw YAMLException("Missing circle footprint \"radius\" in " + name_ +
                        " body");
  }

  b2FixtureDef fixture_def;
  ConfigFootprintCollision(n, fixture_def);

  b2CircleShape shape;
  shape.m_p.Set(x, y);
  shape.m_radius = radius;

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}

void ModelBody::LoadPolygonFootprint(const YAML::Node &footprint_node) {
  const YAML::Node &n = footprint_node;

  std::vector<b2Vec2> points;

  if (n["points"] && n["points"].IsSequence() && n["points"].size() >= 3) {
    for (int i = 0; i < n["points"].size(); i++) {
      YAML::Node np = n["points"][i];

      if (np.IsSequence() && np.size() == 2) {
        b2Vec2 p(np[0].as<double>(), np[1].as<double>());
        points.push_back(p);

      } else {
        throw YAMLException(
            "Missing/invalid polygon footprint \"point\" index=" +
            std::to_string(i) + " in " + name_ +
            " must be a sequence of exactly two items");
      }
    }
  } else {
    throw YAMLException("Missing/invalid polygon footprint \"points\" in " +
                        name_ +
                        " body, must be a sequence with at least 3 "
                        "items");
  }

  b2FixtureDef fixture_def;
  ConfigFootprintCollision(n, fixture_def);

  b2PolygonShape shape;
  shape.Set(points.data(), points.size());

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}

};  // namespace flatland_server
