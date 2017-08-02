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
#include <flatland_server/yaml_reader.h>

namespace flatland_server {

Joint::Joint(b2World *physics_world, Model *model, const std::string &name,
             const Color &color, const b2JointDef &joint_def)
    : physics_world_(physics_world), model_(model), name_(name), color_(color) {
  physics_joint_ = physics_world->CreateJoint(&joint_def);
  physics_joint_->SetUserData(this);
}

Joint::~Joint() { physics_world_->DestroyJoint(physics_joint_); }

Joint *Joint::MakeJoint(b2World *physics_world, Model *model,
                        const YAML::Node &joint_node) {
  YamlReader reader(joint_node);
  Joint *j;

  std::string name = reader.Get<std::string>("name");
  std::string in = "joint " + name;
  std::string type = reader.Get<std::string>("type", in);
  Color color = reader.GetColorOpt("color", Color(1, 1, 1, 0.5), in);
  bool collide_connected = reader.GetOpt<bool>("collide_connected", false, in);

  YamlReader bodies_yr = reader.SubNode("bodies", YamlReader::LIST, in);
  if (bodies_yr.NodeSize() != 2) {
    throw YAMLException("Missing/invalid \"bodies\" in " + name +
                        " joint, must be a sequence of exactly two items");
  }

  Vec2 anchors[2];
  ModelBody *bodies[2];
  for (int i = 0; i < 2; i++) {
    YamlReader body_yr = bodies_yr.SubNode(i, YamlReader::MAP, in);
    std::string name = body_yr.Get<std::string>("name", in);
    anchors[i] = body_yr.GetVec2("anchor", in);
    bodies[i] = model->GetBody(name);

    if (bodies[i] == nullptr) {
      throw YAMLException("Cannot find body with name " + name +
                          " from joint " + name);
    }
  }

  b2Vec2 anchor_A = anchors[0].Box2D();
  b2Vec2 anchor_B = anchors[1].Box2D();
  b2Body *body_A = bodies[0]->physics_body_;
  b2Body *body_B = bodies[1]->physics_body_;

  if (type == "revolute") {
    j = MakeRevoluteJoint(physics_world, model, reader.Node(), name, color, body_A,
                          anchor_A, body_B, anchor_B, collide_connected);
  } else if (type == "weld") {
    j = MakeWeldJoint(physics_world, model, reader.Node(), name, color, body_A,
                      anchor_A, body_B, anchor_B, collide_connected);
  } else {
    throw YAMLException("Invalid joint \"type\" in " + name +
                        " joint, supported joints are: revolute, weld");
  }

  return j;
}

Joint *Joint::MakeRevoluteJoint(b2World *physics_world, Model *model,
                                const YAML::Node &joint_node,
                                const std::string &name, const Color &color,
                                b2Body *body_A, b2Vec2 anchor_A, b2Body *body_B,
                                b2Vec2 anchor_B, bool collide_connected) {
  double upper_limit, lower_limit;
  bool has_limits = false;

  YamlReader reader(joint_node);
  std::string in = "joint " + name;
  std::vector<double> limits = reader.GetListOpt<double>("limits", {}, 2, 2, in);
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
                            const YAML::Node &joint_node,
                            const std::string &name, const Color &color,
                            b2Body *body_A, b2Vec2 anchor_A, b2Body *body_B,
                            b2Vec2 anchor_B, bool collide_connected) {
  YamlReader reader(joint_node);
  std::string in = "joint " + name;
  double angle = 0;
  double frequency = 0;
  double damping = 0;

  reader.GetOpt<double>("angle", 0.0, in);
  reader.GetOpt<double>("frequency", 0.0, in);
  reader.GetOpt<double>("damping", 0.0, in);

  b2WeldJointDef joint_def;
  joint_def.bodyA = body_A;
  joint_def.bodyB = body_B;
  joint_def.localAnchorA = anchor_A;
  joint_def.localAnchorB = anchor_B;
  joint_def.frequencyHz = frequency;
  joint_def.dampingRatio = damping;
  joint_def.referenceAngle = angle;

  return new Joint(physics_world, model, name, color, joint_def);
}

};  // namespace flatland_server
