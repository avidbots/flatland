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
#include <boost/algorithm/string/join.hpp>

namespace flatland_server {

ModelBody::ModelBody(b2World *physics_world, CollisionFilterRegistry *cfr,
                     Model *model, const std::string &name, const Color &color,
                     const Pose &pose, b2BodyType body_type,
                     const YAML::Node &properties, double linear_damping,
                     double angular_damping)
    : Body(physics_world, model, name, color, pose, body_type, properties,
           linear_damping, angular_damping),
      cfr_(cfr) {}

const CollisionFilterRegistry *ModelBody::GetCfr() const { return cfr_; }

ModelBody *ModelBody::MakeBody(b2World *physics_world,
                               CollisionFilterRegistry *cfr, Model *model,
                               YamlReader &body_reader) {
  std::string name = body_reader.Get<std::string>("name");
  body_reader.SetErrorInfo("model " + Q(model->name_), "body " + Q(name));

  Pose pose = body_reader.GetPose("pose", Pose(0, 0, 0));
  Color color = body_reader.GetColor("color", Color(1, 1, 1, 0.5));
  std::string type_str = body_reader.Get<std::string>("type", "dynamic");
  double linear_damping = body_reader.Get("linear_damping", 0.0);
  double angular_damping = body_reader.Get("angular_damping", 0.0);

  b2BodyType type;
  if (type_str == "static") {
    type = b2_staticBody;
  } else if (type_str == "kinematic") {
    type = b2_kinematicBody;
  } else if (type_str == "dynamic") {
    type = b2_dynamicBody;
  } else {
    throw YAMLException("Invalid \"type\" in " + body_reader.entry_location_ +
                        " " + body_reader.entry_name_ +
                        ", must be one of: static, kinematic, dynamic");
  }

  // TODO: Read the model's properties
  ModelBody *m =
      new ModelBody(physics_world, cfr, model, name, color, pose, type,
                    YAML::Node(), linear_damping, angular_damping);

  try {
    YamlReader footprints_node =
        body_reader.Subnode("footprints", YamlReader::LIST);
    body_reader.EnsureAccessedAllKeys();

    m->LoadFootprints(footprints_node);
  } catch (const YAMLException &e) {
    delete m;
    throw e;
  }

  return m;
}

void ModelBody::LoadFootprints(YamlReader &footprints_reader) {
  for (int i = 0; i < footprints_reader.NodeSize(); i++) {
    YamlReader reader = footprints_reader.Subnode(i, YamlReader::MAP);

    std::string type = reader.Get<std::string>("type");
    if (type == "circle") {
      LoadCircleFootprint(reader);
    } else if (type == "polygon") {
      LoadPolygonFootprint(reader);
    } else {
      throw YAMLException("Invalid footprint \"type\" in " +
                          reader.entry_location_ + " " + reader.entry_name_ +
                          ", support footprints are: circle, polygon");
    }

    reader.EnsureAccessedAllKeys();
  }
}

void ModelBody::ConfigFootprintDef(YamlReader &footprint_reader,
                                   b2FixtureDef &fixture_def) {
  // configure physics properties
  fixture_def.density = footprint_reader.Get<float>("density");
  fixture_def.friction = footprint_reader.Get<float>("friction", 0.0);
  fixture_def.restitution = footprint_reader.Get<float>("restitution", 0.0);

  // config collision properties
  fixture_def.isSensor = footprint_reader.Get<bool>("sensor", false);
  fixture_def.filter.groupIndex = 0;

  std::vector<std::string> layers =
      footprint_reader.GetList<std::string>("layers", {"all"}, -1, -1);
  std::vector<std::string> invalid_layers;
  fixture_def.filter.categoryBits =
      cfr_->GetCategoryBits(layers, &invalid_layers);

  if (!invalid_layers.empty()) {
    throw YAMLException("Invalid footprint \"layers\" in " +
                        footprint_reader.entry_location_ + " " +
                        footprint_reader.entry_name_ + ", {" +
                        boost::algorithm::join(invalid_layers, ",") +
                        "} layer(s) does not exist");
  }

  bool collision = footprint_reader.Get<bool>("collision", true);
  if (collision) {
    // b2d docs: maskBits are "I collide with" bitmask
    fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
  } else {
    // "I will collide with nothing"
    fixture_def.filter.maskBits = 0;
  }
}

void ModelBody::LoadCircleFootprint(YamlReader &footprint_reader) {
  Vec2 center = footprint_reader.GetVec2("center", Vec2(0, 0));
  double radius = footprint_reader.Get<double>("radius");

  b2FixtureDef fixture_def;
  ConfigFootprintDef(footprint_reader, fixture_def);

  b2CircleShape shape;
  shape.m_p.Set(center.x, center.y);
  shape.m_radius = radius;

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}

void ModelBody::LoadPolygonFootprint(YamlReader &footprint_reader) {
  std::vector<b2Vec2> points =
      footprint_reader.GetList<b2Vec2>("points", 3, b2_maxPolygonVertices);

  b2FixtureDef fixture_def;
  ConfigFootprintDef(footprint_reader, fixture_def);

  b2PolygonShape shape;
  shape.Set(points.data(), points.size());

  fixture_def.shape = &shape;
  physics_body_->CreateFixture(&fixture_def);
}
};
