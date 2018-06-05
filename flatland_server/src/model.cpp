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
    : Entity(physics_world, name),
      namespace_(ns),
      cfr_(cfr),
      viz_name_("model/" + name_) {}

Model::~Model() {
  for (unsigned int i = 0; i < joints_.size(); i++) {
    delete joints_[i];
  }

  for (unsigned int i = 0; i < bodies_.size(); i++) {
    delete bodies_[i];
  }

  // No need to destroy joints since destruction of model will destroy the
  // joint, the creation of a joint must always have bodies attached to it

  // clear visualization
  DebugVisualization::Get().Reset(viz_name_);
}

Model *Model::MakeModel(b2World *physics_world, CollisionFilterRegistry *cfr,
                        const std::string &model_yaml_path,
                        const std::string &ns, const std::string &name) {
  YamlReader reader(model_yaml_path);
  reader.SetErrorInfo("model " + Q(name));

  Model *m = new Model(physics_world, cfr, ns, name);

  m->plugins_reader_ = reader.SubnodeOpt("plugins", YamlReader::LIST);

  try {
    YamlReader bodies_reader = reader.Subnode("bodies", YamlReader::LIST);
    YamlReader joints_reader = reader.SubnodeOpt("joints", YamlReader::LIST);
    reader.EnsureAccessedAllKeys();

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
                        " model, must a be list of bodies of at least size 1");
  } else {
    for (int i = 0; i < bodies_reader.NodeSize(); i++) {
      YamlReader body_reader = bodies_reader.Subnode(i, YamlReader::MAP);
      if (!body_reader.Get<bool>("enabled", "true")) {
        ROS_INFO_STREAM("Body "
                        << Q(name_) << "."
                        << body_reader.Get<std::string>("name", "unnamed")
                        << " disabled");
        continue;
      }
      ModelBody *b =
          ModelBody::MakeBody(physics_world_, cfr_, this, body_reader);
      bodies_.push_back(b);

      // ensure body is not a duplicate
      if (std::count_if(bodies_.begin(), bodies_.end(),
                        [&](Body *i) { return i->name_ == b->name_; }) > 1) {
        throw YAMLException("Invalid \"bodies\" in " + Q(name_) +
                            " model, body with name " + Q(b->name_) +
                            " already exists");
      }
    }
  }
}

void Model::LoadJoints(YamlReader &joints_reader) {
  if (!joints_reader.IsNodeNull()) {
    for (int i = 0; i < joints_reader.NodeSize(); i++) {
      YamlReader joint_reader = joints_reader.Subnode(i, YamlReader::MAP);
      if (!joint_reader.Get<bool>("enabled", "true")) {
        ROS_INFO_STREAM("Joint "
                        << Q(name_) << "."
                        << joint_reader.Get<std::string>("name", "unnamed")
                        << " disabled");
        continue;
      }
      Joint *j = Joint::MakeJoint(physics_world_, this, joint_reader);
      joints_.push_back(j);

      // ensure joint is not a duplicate
      if (std::count_if(joints_.begin(), joints_.end(),
                        [&](Joint *i) { return i->name_ == j->name_; }) > 1) {
        throw YAMLException("Invalid \"joints\" in " + Q(name_) +
                            " model, joint with name " + Q(j->name_) +
                            " already exists");
      }
    }
  }
}

ModelBody *Model::GetBody(const std::string &name) {
  for (unsigned int i = 0; i < bodies_.size(); i++) {
    if (bodies_[i]->name_ == name) {
      return bodies_[i];
    }
  }
  return nullptr;
}

Joint *Model::GetJoint(const std::string &name) {
  for (unsigned int i = 0; i < joints_.size(); i++) {
    if (joints_[i]->name_ == name) {
      return joints_[i];
    }
  }
  return nullptr;
}

const std::vector<ModelBody *> &Model::GetBodies() { return bodies_; }

const std::vector<Joint *> &Model::GetJoints() { return joints_; }

const std::string &Model::GetNameSpace() const { return namespace_; }

std::string Model::NameSpaceTF(const std::string &frame_id) const {
  // case: "global" namespace: don't prefix, strip leading slash
  if (frame_id.substr(0, 1) == "/") {
    return std::string(frame_id, 1,
                       std::string::npos);  // Strip the leading '/'
  } else {  // case: "local" namespace: prepend namespace
    if (namespace_.length() > 0) {
      return namespace_ + "_" + frame_id;
    } else {
      return frame_id;
    }
  }
}

std::string Model::NameSpaceTopic(const std::string &topic) const {
  // We don't actually want the topic to be "global" in case flatland is itself
  // namespaced, so strip leading slash
  if (topic.substr(0, 1) == "/") {
    return std::string(topic, 1, std::string::npos);  // Strip the leading '/'
  } else {  // case: "local" namespace: prepend namespace/
    if (namespace_.length() > 0) {
      return namespace_ + "/" + topic;
    } else {
      return topic;
    }
  }
}

const std::string &Model::GetName() const { return name_; }

const CollisionFilterRegistry *Model::GetCfr() const { return cfr_; }

void Model::SetPose(const Pose &pose) {
  // Grab first (root?) body transform
  RotateTranslate root_body_transform =
      Geometry::CreateTransform(bodies_[0]->physics_body_->GetPosition().x,
                                bodies_[0]->physics_body_->GetPosition().y,
                                bodies_[0]->physics_body_->GetAngle());

  // Inverse transform all bodies by this to reset their poses
  for (unsigned int i = 0; i < bodies_.size(); i++) {
    bodies_[i]->physics_body_->SetTransform(
        Geometry::InverseTransform(bodies_[i]->physics_body_->GetPosition(),
                                   root_body_transform),
        0.0);
  }

  // Apply new desired pose in world coordinates
  TransformAll(pose);
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

  for (unsigned int i = 0; i < bodies_.size(); i++) {
    bodies_[i]->physics_body_->SetTransform(
        Geometry::Transform(bodies_[i]->physics_body_->GetPosition(), tf),
        bodies_[i]->physics_body_->GetAngle() + pose_delta.theta);
  }
}

void Model::DebugVisualize() const {
  DebugVisualization::Get().Reset(viz_name_);

  for (const auto &body : bodies_) {
    DebugVisualization::Get().Visualize(viz_name_, body->physics_body_,
                                        body->color_.r, body->color_.g,
                                        body->color_.b, body->color_.a);
  }

  for (const auto &joint : joints_) {
    DebugVisualization::Get().Visualize(viz_name_, joint->physics_joint_,
                                        joint->color_.r, joint->color_.g,
                                        joint->color_.b, joint->color_.a);
  }
}

void Model::DebugOutput() const {
  ROS_DEBUG_NAMED("Model",
                  "Model %p: physics_world(%p) name(%s) namespace(%s) "
                  "num_bodies(%lu) num_joints(%lu)",
                  this, physics_world_, name_.c_str(), namespace_.c_str(),
                  bodies_.size(), joints_.size());

  for (const auto &body : bodies_) {
    body->DebugOutput();
  }

  for (const auto &joint : joints_) {
    joint->DebugOutput();
  }
}

void Model::DumpBox2D() const {
  for (const auto &body : bodies_) {
    b2Log("BODY %p name=%s box2d_body=%p model=%p model_name=%s\n", body,
          body->name_.c_str(), body->physics_body_, this, name_.c_str());
    body->physics_body_->Dump();
  }

  for (const auto &joint : joints_) {
    Body *body_A =
        static_cast<Body *>(joint->physics_joint_->GetBodyA()->GetUserData());
    Body *body_B =
        static_cast<Body *>(joint->physics_joint_->GetBodyB()->GetUserData());
    b2Log(
        "JOINT %p name=%s  box2d_joint=%p model=%p model_name=%s "
        "body_A(%p %s) body_B(%p %s)\n",
        joint, joint->name_.c_str(), joint->physics_joint_, this, name_.c_str(),
        body_A, body_A->name_.c_str(), body_B, body_B->name_.c_str());
    joint->physics_joint_->Dump();
  }
}
};  // namespace flatland_server
