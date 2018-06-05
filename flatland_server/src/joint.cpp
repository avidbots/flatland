/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 joint.cpp
 * @brief	 Implements Joint
 * @author   Chunshang Li
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
#include <flatland_server/joint.h>
#include <ros/ros.h>

namespace flatland_server {

Joint::Joint(b2World *physics_world, Model *model, const std::string &name,
             const Color &color, const b2JointDef &joint_def)
    : model_(model), name_(name), physics_world_(physics_world), color_(color) {
  physics_joint_ = physics_world->CreateJoint(&joint_def);
  physics_joint_->SetUserData(this);
}

Joint::~Joint() { physics_world_->DestroyJoint(physics_joint_); }

Model *Joint::GetModel() { return model_; }

const std::string &Joint::GetName() const { return name_; }

const Color &Joint::GetColor() const { return color_; }

void Joint::SetColor(const Color &color) { color_ = color; }

b2Joint *Joint::GetPhysicsJoint() { return physics_joint_; }

b2World *Joint::GetphysicsWorld() { return physics_world_; }

Joint *Joint::MakeJoint(b2World *physics_world, Model *model,
                        YamlReader &joint_reader) {
  Joint *j;

  std::string name = joint_reader.Get<std::string>("name");
  joint_reader.SetErrorInfo("model " + Q(model->name_), "joint " + Q(name));

  std::string type = joint_reader.Get<std::string>("type");
  Color color = joint_reader.GetColor("color", Color(1, 1, 1, 0.5));
  bool collide_connected = joint_reader.Get<bool>("collide_connected", false);

  YamlReader bodies_reader = joint_reader.Subnode("bodies", YamlReader::LIST);
  if (bodies_reader.NodeSize() != 2) {
    throw YAMLException("Invalid \"bodies\" in " +
                        bodies_reader.entry_location_ +
                        ", must be a sequence of exactly two items");
  }

  Vec2 anchors[2];
  ModelBody *bodies[2];
  for (unsigned int i = 0; i < 2; i++) {
    YamlReader body_reader = bodies_reader.Subnode(i, YamlReader::MAP);
    std::string body_name = body_reader.Get<std::string>("name");
    anchors[i] = body_reader.GetVec2("anchor");
    bodies[i] = model->GetBody(body_name);

    if (bodies[i] == nullptr) {
      throw YAMLException("Cannot find body with name " + Q(body_name) +
                          " in " + body_reader.entry_location_ + " " +
                          body_reader.entry_name_);
    }
  }

  b2Vec2 anchor_A = anchors[0].Box2D();
  b2Vec2 anchor_B = anchors[1].Box2D();
  b2Body *body_A = bodies[0]->physics_body_;
  b2Body *body_B = bodies[1]->physics_body_;

  if (type == "revolute") {
    j = MakeRevoluteJoint(physics_world, model, joint_reader, name, color,
                          body_A, anchor_A, body_B, anchor_B,
                          collide_connected);
  } else if (type == "weld") {
    j = MakeWeldJoint(physics_world, model, joint_reader, name, color, body_A,
                      anchor_A, body_B, anchor_B, collide_connected);
  } else {
    throw YAMLException(
        "Invalid joint \"type\" in " + joint_reader.entry_location_ + " " +
        joint_reader.entry_name_ + ", supported joints are: revolute, weld");
  }

  try {
    joint_reader.EnsureAccessedAllKeys();
  } catch (const YAMLException &e) {
    delete j;
    throw e;
  }
  return j;
}

Joint *Joint::MakeRevoluteJoint(b2World *physics_world, Model *model,
                                YamlReader &joint_reader,
                                const std::string &name, const Color &color,
                                b2Body *body_A, b2Vec2 anchor_A, b2Body *body_B,
                                b2Vec2 anchor_B, bool collide_connected) {
  double upper_limit, lower_limit;
  bool has_limits = false;

  std::vector<double> limits = joint_reader.GetList<double>("limits", {}, 2, 2);
  if (limits.size() == 2) {
    lower_limit = limits[0];
    upper_limit = limits[1];
    has_limits = true;
  }

  b2RevoluteJointDef joint_def;
  joint_def.bodyA = body_A;
  joint_def.bodyB = body_B;
  joint_def.localAnchorA = anchor_A;
  joint_def.localAnchorB = anchor_B;
  joint_def.collideConnected = collide_connected;

  if (has_limits) {
    joint_def.lowerAngle = lower_limit;
    joint_def.upperAngle = upper_limit;
    joint_def.enableLimit = true;
  } else {
    joint_def.enableLimit = false;
  }

  return new Joint(physics_world, model, name, color, joint_def);
}

Joint *Joint::MakeWeldJoint(b2World *physics_world, Model *model,
                            YamlReader &joint_reader, const std::string &name,
                            const Color &color, b2Body *body_A, b2Vec2 anchor_A,
                            b2Body *body_B, b2Vec2 anchor_B,
                            bool collide_connected) {
  double angle = joint_reader.Get<double>("angle", 0.0);
  double frequency = joint_reader.Get<double>("frequency", 0.0);
  double damping = joint_reader.Get<double>("damping", 0.0);

  b2WeldJointDef joint_def;
  joint_def.bodyA = body_A;
  joint_def.bodyB = body_B;
  joint_def.localAnchorA = anchor_A;
  joint_def.localAnchorB = anchor_B;
  joint_def.frequencyHz = frequency;
  joint_def.dampingRatio = damping;
  joint_def.referenceAngle = angle;
  joint_def.collideConnected = collide_connected;

  return new Joint(physics_world, model, name, color, joint_def);
}

void Joint::DebugOutput() const {
  b2Joint *j = physics_joint_;
  Body *body_A = static_cast<Body *>(j->GetBodyA()->GetUserData());
  Body *body_B = static_cast<Body *>(j->GetBodyB()->GetUserData());

  ROS_DEBUG_NAMED("Joint",
                  "Joint %p: model(%p, %s) name(%s) color(%f,%f,%f,%f) "
                  "physics_joint(%p) body_A(%p, %s) anchor_A_world(%f, %f) "
                  "body_B(%p, %s) anchor_B_world(%f, %f)",
                  this, model_, model_->name_.c_str(), name_.c_str(), color_.r,
                  color_.g, color_.b, color_.a, physics_joint_, body_A,
                  body_A->name_.c_str(), j->GetAnchorA().x, j->GetAnchorA().y,
                  body_B, body_B->name_.c_str(), j->GetAnchorB().x,
                  j->GetAnchorB().y);
}

};  // namespace flatland_server
