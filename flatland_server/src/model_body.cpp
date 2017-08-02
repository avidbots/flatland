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
#include <flatland_server/yaml_reader.h>
#include <boost/algorithm/string/join.hpp>

namespace flatland_server {

ModelBody::ModelBody(b2World *physics_world, CollisionFilterRegistry *cfr,
                     Model *model, const std::string &name, const Color &color,
                     const Pose &origin, b2BodyType body_type,
                     double linear_damping, double angular_damping)
    : Body(physics_world, model, name, color, origin, body_type, linear_damping,
           angular_damping),
      cfr_(cfr) {}

ModelBody *ModelBody::MakeBody(b2World *physics_world,
                               CollisionFilterRegistry *cfr, Model *model,
                               const YAML::Node &body_node) {
  YamlReader reader(body_node);

  std::string name = reader.Get<std::string>("name");
  std::string in = "body " + name;
  Pose origin = reader.GetPose("origin", Pose(0, 0, 0), in);
  Color color = reader.GetColor("color", Color(1, 1, 1, 0.5), in);
  std::string type_str = reader.Get<std::string>("type", in);
  double linear_damping = reader.Get("linear_damping", 0.0, in);
  double angular_damping = reader.Get("angular_damping", 0.0, in);

  b2BodyType type;
  if (type_str == "static") {
    type = b2_staticBody;
  } else if (type_str == "kinematic") {
    type = b2_kinematicBody;
  } else if (type_str == "dynamic") {
    type = b2_dynamicBody;
  } else {
    throw YAMLException("Invalid \"type\" in " + name +
                        " body, must be one of: static, kinematic, dynamic");
  }

  ModelBody *m = new ModelBody(physics_world, cfr, model, name, color, origin,
                               type, linear_damping, angular_damping);

  try {
    m->LoadFootprints(
        reader.SubNode("footprints", YamlReader::LIST, in).Node());
  } catch (const YAML::Exception &e) {
    delete m;
    throw YAMLException(e);
  }

  return m;
}

void ModelBody::LoadFootprints(const YAML::Node &footprints_node) {
  YamlReader footprints_reader(footprints_node);
  std::string in = "body " + name_;

  for (int i = 0; i < footprints_reader.NodeSize(); i++) {
    YamlReader reader = footprints_reader.SubNode(i, YamlReader::MAP, in);

    std::string type = reader.Get<std::string>("type", in);
    if (type == "circle") {
      LoadCircleFootprint(reader.Node());
    } else if (type == "polygon") {
      LoadPolygonFootprint(reader.Node());
    } else {
      throw YAMLException("Invalid footprint \"type\" in " + name_ +
                          " body, support footprints are: circle, polygon");
    }
  }
}

void ModelBody::ConfigFootprintDef(const YAML::Node &footprint_node,
                                   b2FixtureDef &fixture_def) {
  YamlReader reader(footprint_node);
  std::string in = "body " + name_;

  // configure physics properties
  fixture_def.density = reader.Get<float>("density", 0.0, in);
  fixture_def.friction = reader.Get<float>("friction", 0.0, in);
  fixture_def.restitution = reader.Get<float>("restitution", 0.0, in);

  // config collision properties
  fixture_def.isSensor = reader.Get<bool>("restitution", false, in);
  bool self_collide = reader.Get<bool>("self_collide", false, in);
  std::vector<std::string> layers =
      reader.GetList<std::string>("layers", {"all"}, -1, -1, in);

  if (layers.size() == 1 && layers[0] == "all") {
    layers.clear();
    cfr_->ListAllLayers(layers);
  }

  if (self_collide) {
    fixture_def.filter.groupIndex = cfr_->RegisterCollide();
  } else {
    fixture_def.filter.groupIndex =
        (dynamic_cast<Model *>(entity_))->no_collide_group_index_;
  }

  fixture_def.filter.categoryBits = 0x0;

  std::vector<std::string> failed_layers;
  uint16_t category_bits = cfr_->GetCategoryBits(layers, &failed_layers);

  if (!failed_layers.empty()) {
    throw YAMLException("Invalid footprint \"layer\" in " + name_ + " body, {" +
                        boost::algorithm::join(failed_layers, ",") +
                        "} layer(s) does not exist");
  }

  fixture_def.filter.categoryBits = category_bits;
  fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
}

void ModelBody::LoadCircleFootprint(const YAML::Node &footprint_node) {
  YamlReader reader(footprint_node);
  std::string in = "body " + name_;

  Vec2 center = reader.GetVec2("center", in);
  double radius = reader.Get<double>("radius", in);

  b2FixtureDef fixture_def;
  ConfigFootprintDef(reader.Node(), fixture_def);

  b2CircleShape shape;
  shape.m_p.Set(center.x, center.y);
  shape.m_radius = radius;

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}

void ModelBody::LoadPolygonFootprint(const YAML::Node &footprint_node) {
  YamlReader reader(footprint_node);
  std::vector<b2Vec2> points = reader.GetList<b2Vec2>(
      "points", 3, b2_maxPolygonVertices, "body " + name_);

  b2FixtureDef fixture_def;
  ConfigFootprintDef(reader.Node(), fixture_def);

  b2PolygonShape shape;
  shape.Set(points.data(), points.size());

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}
};
