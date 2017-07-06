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

#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/model.h>

namespace flatland_server {

Model::Model(b2World *physics_world, CollisionFilterRegistry *cfr,
             const std::string &name)
    : Entity(physics_world), name_(name), cfr_(cfr) {
  no_collide_group_index_ = cfr->RegisterNoCollide();
}

Model::~Model() {
  for (int i = 0; i < bodies_.size(); i++) {
    delete bodies_[i];
  }

  // No need to destroy joints since destruction of model will destroy the
  // joint, the creation of a joint must always have bodies attached to it
}

Model *Model::MakeModel(b2World *physics_world, CollisionFilterRegistry *cfr,
                        const std::string &name,
                        const std::string &model_yaml_path) {
  YAML::Node model_node;
  try {
    model_node = YAML::LoadFile(model_yaml_path);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + model_yaml_path, e);
  }

  Model *m = new Model(physics_world, cfr, name);

  // it is okay to have no plugins
  if (model_node["plugins"] && !model_node["plugins"].IsSequence()) {
    throw YAMLException("Invalid \"plugins\" in " + name +
                        " model, not a list");
  } else if (model_node["plugins"] && !model_node["plugins"].IsSequence()) {
    m->plugins_node_ = model_node["plugins"];
  }

  try {
    m->LoadBodies(model_node["bodies"]);
    m->LoadJoints(model_node["joints"]);
  } catch (const YAML::Exception &e) {
    delete m;
    throw YAMLException(e);
  }

  return m;
}

void Model::LoadBodies(const YAML::Node &bodies_node) {
  if (!bodies_node || !bodies_node.IsSequence() || bodies_node.size() <= 0) {
    throw YAMLException("Invalid \"bodies\" in " + name_ +
                        " model, "
                        "must a be list of bodies of at least size 1");
  } else {
    for (const auto &body_node : bodies_node) {
      ModelBody *b = ModelBody::MakeBody(physics_world_, cfr_, this, body_node);
      bodies_.push_back(b);
    }
  }
}

void Model::LoadJoints(const YAML::Node &joints_node) {
  if (joints_node && !joints_node.IsSequence()) {
    // if joints exists and it is not a sequence, it is okay to have no joints
    throw YAMLException("Invalid \"joints\" in " + name_ + " model");
  } else if (joints_node) {
    for (const auto &joint_node : joints_node) {
      Joint *j = Joint::MakeJoint(physics_world_, this, joint_node);
      joints_.push_back(j);
    }
  }
}

ModelBody *Model::GetBody(const std::string &name) {
  for (int i = 0; i < bodies_.size(); i++) {
    if (bodies_[i]->name_ == name) {
      return bodies_[i];
    }
  }
  return nullptr;
}

void Model::TransformAll(const std::array<double, 3> &pose) {
  //     --                --   --                --
  //     | cos(a) -sin(a) x |   | cos(b) -sin(b) u |
  //     | sin(a)  cos(a) y | x | sin(b)  cos(b) v |
  //     | 0       0      1 |   | 0       0      1 |
  //     --                --   --                --
  //       --                                          --
  //       | cos(a+b) -sin(a+b) x + u*cos(a) - v*sin(a) |
  //     = | sin(a+b)  cos(a+b) y + u*sin(a) + v*cos(a) |
  //       | 0         0        1                       |
  //       --                                          --

  RotateTranslate tf = Geometry::CreateTransform(pose[0], pose[1], pose[2]);

  for (int i = 0; i < bodies_.size(); i++) {
    bodies_[i]->physics_body_->SetTransform(
        Geometry::Transform(bodies_[i]->physics_body_->GetPosition(), tf),
        bodies_[i]->physics_body_->GetAngle() + pose[2]);
  }
}

};  // namespace flatland_server
