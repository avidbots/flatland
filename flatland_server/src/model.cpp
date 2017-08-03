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

#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/model.h>

namespace flatland_server {

Model::Model(b2World *physics_world, CollisionFilterRegistry *cfr,
             const std::string &ns, const std::string &name)
    : Entity(physics_world, name), namespace_(ns), cfr_(cfr) {
  no_collide_group_index_ = cfr->RegisterNoCollide();
}

Model::~Model() {
  for (int i = 0; i < joints_.size(); i++) {
    delete joints_[i];
  }

  for (int i = 0; i < bodies_.size(); i++) {
    delete bodies_[i];
  }

  // No need to destroy joints since destruction of model will destroy the
  // joint, the creation of a joint must always have bodies attached to it
}

Model *Model::MakeModel(b2World *physics_world, CollisionFilterRegistry *cfr,
                        const std::string &model_yaml_path,
                        const std::string &ns, const std::string &name) {
  YamlReader reader(model_yaml_path);
  reader.SetErrorInfo("model " + Q(name));

  Model *m = new Model(physics_world, cfr, ns, name);

  m->plugins_reader_ = reader.SubNodeOpt("plugins", YamlReader::LIST);

  try {
    YamlReader bodies_reader = reader.SubNode("bodies", YamlReader::LIST);
    YamlReader joints_reader = reader.SubNodeOpt("joints", YamlReader::LIST);
    m->LoadBodies(bodies_reader);
    m->LoadJoints(joints_reader);
  } catch (const YAMLException &e) {
    delete m;
    throw e;
  }

  return m;
}

void Model::LoadBodies(YamlReader &bodies_reader) {
  if (bodies_reader.NodeSize() <= 0) {
    throw YAMLException("Invalid \"bodies\" in " + Q(name_) +
                        " model, "
                        "must a be list of bodies of at least size 1");
  } else {
    for (int i = 0; i < bodies_reader.NodeSize(); i++) {
      YamlReader body_reader = bodies_reader.SubNode(i, YamlReader::MAP);
      ModelBody *b =
          ModelBody::MakeBody(physics_world_, cfr_, this, body_reader);
      bodies_.push_back(b);
    }
  }
}

void Model::LoadJoints(YamlReader &joints_reader) {
  if (!joints_reader.IsNodeNull()) {
    for (int i = 0; i < joints_reader.NodeSize(); i++) {
      YamlReader joint_reader = joints_reader.SubNode(i, YamlReader::MAP);
      Joint *j = Joint::MakeJoint(physics_world_, this, joint_reader);
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

void Model::TransformAll(const Pose &pose_delta) {
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

  RotateTranslate tf =
      Geometry::CreateTransform(pose_delta.x, pose_delta.y, pose_delta.theta);

  for (int i = 0; i < bodies_.size(); i++) {
    bodies_[i]->physics_body_->SetTransform(
        Geometry::Transform(bodies_[i]->physics_body_->GetPosition(), tf),
        bodies_[i]->physics_body_->GetAngle() + pose_delta.theta);
  }
}

void Model::DebugVisualize() {
  std::string name = "model/" + name_;
  DebugVisualization::Get().Reset(name);

  for (auto &body : bodies_) {
    DebugVisualization::Get().Visualize(name, body->physics_body_,
                                        body->color_.r, body->color_.g,
                                        body->color_.b, body->color_.a);
  }

  for (auto &joint : joints_) {
    DebugVisualization::Get().Visualize(name, joint->physics_joint_,
                                        joint->color_.r, joint->color_.g,
                                        joint->color_.b, joint->color_.a);
  }
}

};  // namespace flatland_server
