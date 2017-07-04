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

namespace flatland_server {

Joint::Joint(b2World *physics_world, Model *model, const std::string &name,
             const b2JointDef &joint_def)
    : physics_world_(physics_world), model_(model), name_(name) {
  physics_joint_ = physics_world->CreateJoint(&joint_def);
  physics_joint_->SetUserData(this);
}

Joint::~Joint() { physics_world_->DestroyJoint(physics_joint_); }

Joint *Joint::MakeJoint(b2World *physics_world, Model *model,
                        const YAML::Node &joint_node) {
  const YAML::Node &n = joint_node;

  std::string name;

  if (n["name"]) {
    name = n["name"].as<std::string>();
  } else {
    throw YAMLException("Missing a joint name");
  }

  Joint *j;
  std::string type;

  if (n["type"]) {
    type = n["type"].as<std::string>();
  } else {
    throw YAMLException("Missing \"type\" in " + name + " joint");
  }

  if (type == "revolute") {
    j = MakeRevoluteJoint(physics_world, model, n, name);
  } else if (type == "weld") {
    j = MakeWeldJoint(physics_world, model, n, name);
  } else {
    throw YAMLException("Invalid joint \"type\" in " + name +
                        " joint, support joints are: revolute, weld");
  }

  return j;
}

Joint *Joint::MakeRevoluteJoint(b2World *physics_world, Model *model,
                                const YAML::Node &joint_node,
                                const std::string &name) {
  const YAML::Node &n = joint_node;
  double upper_limit, lower_limit;
  bool has_limits = false;
  bool collide_connected;
  b2Body *body_A, *body_B;
  b2Vec2 anchor_A, anchor_B;

  if (n["limits"] && (!n["limits"].IsSequence() || !n["limits"].size() == 2)) {
    throw YAMLException("Invalid \"limits\" in " + name +
                        " joint, must be "
                        "a sequence of exactly two items");
  } else {
    lower_limit = n["limits"][0].as<double>();
    upper_limit = n["limits"][1].as<double>();
    has_limits = true;
  }

  ParseJointCommon(model, n, name, body_A, anchor_A, body_B, anchor_B,
                   collide_connected);

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

  return new Joint(physics_world, model, name, joint_def);
}

Joint *Joint::MakeWeldJoint(b2World *physics_world, Model *model,
                            const YAML::Node &joint_node,
                            const std::string &name) {
  const YAML::Node &n = joint_node;

  double angle = 0;
  double frequency = 0;
  double damping = 0;
  b2Body *body_A, *body_B;
  b2Vec2 anchor_A, anchor_B;
  bool collide_connected;

  if (n["angle"]) {
    angle = n["angle"].as<double>();
  }

  if (n["frequency"]) {
    frequency = n["angle"].as<double>();
  }

  if (n["damping"]) {
    damping = n["damping"].as<double>();
  }

  ParseJointCommon(model, n, name, body_A, anchor_A, body_B, anchor_B,
                   collide_connected);

  b2WeldJointDef joint_def;
  joint_def.bodyA = body_A;
  joint_def.bodyB = body_B;
  joint_def.localAnchorA = anchor_A;
  joint_def.localAnchorB = anchor_B;
  joint_def.frequencyHz = frequency;
  joint_def.dampingRatio = damping;
  joint_def.referenceAngle = angle;

  return new Joint(physics_world, model, name, joint_def);
}

void Joint::ParseJointCommon(Model *model, const YAML::Node &joint_node,
                             const std::string &joint_name, b2Body *&body_A,
                             b2Vec2 &anchor_A, b2Body *&body_B,
                             b2Vec2 &anchor_B, bool &collide_connected) {
  const YAML::Node &n = joint_node;
  b2Vec2 anchors[2];
  ModelBody *bodies[2];
  collide_connected = false;

  if (n["collide_connected"]) {
    collide_connected = n["collide_connected"].as<bool>();
  }

  if (n["bodies"] && !n["bodies"].IsSequence() && n["bodies"].size() == 2) {
    for (int i = 0; i < 2; i++) {
      YAML::Node body = n["bodies"][i];
      if (body["name"]) {
        std::string name = body["name"].as<std::string>();

        bodies[i] = model->GetBody(name);

        if (body == NULL) {
          throw YAMLException("Cannot find body with name " + name +
                              " in joint " + joint_name);
        }
      } else {
        throw YAMLException("Missing body \"name\" in " + joint_name +
                            " joint "
                            "body index=" +
                            std::to_string(i));
      }

      if (body["anchor"] && body["anchor"].IsSequence() &&
          body["anchor"].size() == 2) {
        double x = body["anchor"][0].as<double>();
        double y = body["anchor"][1].as<double>();

        anchors[0].Set(x, y);
      } else {
        throw YAMLException("Missing/invalid body \"anchor\" in " + joint_name +
                            " joint body index=" + std::to_string(i) +
                            ". must"
                            " be a sequence of exactly two numbers");
      }
    }

    anchor_A = anchors[0];
    anchor_B = anchors[1];
    body_A = bodies[0]->physics_body_;
    body_B = bodies[1]->physics_body_;

  } else {
    throw YAMLException("Missing/invalid \"bodies\" in " + joint_name +
                        " joint, must"
                        " be a sequence of exactly two items");
  }
}

};  // namespace flatland_server
