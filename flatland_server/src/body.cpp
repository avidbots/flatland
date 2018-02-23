/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 body.cpp
 * @brief	 implements flatland body
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

#include <flatland_server/body.h>
#include <ros/ros.h>

namespace flatland_server {

Body::Body(b2World *physics_world, Entity *entity, const std::string &name,
           const Color &color, const Pose &pose, b2BodyType body_type,
           const YAML::Node &properties, double linear_damping,
           double angular_damping)
    : entity_(entity), name_(name), color_(color), properties_(properties) {
  b2BodyDef body_def;
  body_def.type = body_type;
  body_def.position.Set(pose.x, pose.y);
  body_def.angle = pose.theta;
  body_def.linearDamping = linear_damping;
  body_def.angularDamping = angular_damping;

  physics_body_ = physics_world->CreateBody(&body_def);
  physics_body_->SetUserData(this);
}

Body::~Body() {
  if (physics_body_) {
    physics_body_->GetWorld()->DestroyBody(physics_body_);
  }
}

int Body::GetFixturesCount() const {
  int count = 0;
  for (b2Fixture *f = physics_body_->GetFixtureList(); f; f = f->GetNext()) {
    count++;
  }

  return count;
}

Entity *Body::GetEntity() { return entity_; }

const std::string &Body::GetName() const { return name_; }

b2Body *Body::GetPhysicsBody() { return physics_body_; }

const Color &Body::GetColor() const { return color_; }

void Body::SetColor(const Color &color) { color_ = color; }

void Body::DebugOutput() const {
  ROS_DEBUG_NAMED(
      "Body",
      "Body %p: entity(%p, %s) name(%s) color(%f,%f,%f,%f) "
      "physics_body(%p) num_fixtures(%d) type(%d) pose(%f, %f, %f) "
      "angular_damping(%f) linear_damping(%f)",
      this, entity_, entity_->name_.c_str(), name_.c_str(), color_.r, color_.g,
      color_.b, color_.a, physics_body_, GetFixturesCount(),
      physics_body_->GetType(), physics_body_->GetPosition().x,
      physics_body_->GetPosition().y, physics_body_->GetAngle(),
      physics_body_->GetAngularDamping(), physics_body_->GetLinearDamping());
}

};  // namespace flatland_server
