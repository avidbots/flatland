/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  bumper.h
 * @brief   Bumper plugin
 * @author  Chunshang Li
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

#include <flatland_msgs/ContactState.h>
#include <flatland_msgs/ContactsState.h>
#include <flatland_plugins/bumper.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/timekeeper.h>
#include <pluginlib/class_list_macros.h>

using namespace flatland_server;

namespace flatland_plugins {

Bumper::ContactState::ContactState() { Reset(); }

void Bumper::ContactState::Reset() {
  num_count = 0;
  sum_normal_impulses[0] = 0;
  sum_normal_impulses[1] = 0;
  sum_tangential_impulses[0] = 0;
  sum_tangential_impulses[1] = 0;
}

void Bumper::OnInitialize(const YAML::Node &config) {
  world_frame_id_ = "map";
  update_rate_ = std::numeric_limits<double>::infinity();

  if (config["world_frame_id"]) {
    world_frame_id_ = config["world_frame_id"].as<std::string>();
  }

  if (config["bodies"] && config["bodies"].IsSequence()) {
    for (int i = 0; i < config["bodies"].size(); i++) {
      std::string body_name = config["bodies"][i].as<std::string>();

      Body *body = model_->GetBody(body_name);

      if (body == nullptr) {
        throw YAMLException("Cannot find body with name " + body_name);
      } else {
        bodies_.push_back(body);
      }
    }
  } else {
    throw YAMLException(
        "Missing/invalid \"bodies\" param, must be a list of body names");
  }

  if (config["update_rate"]) {
    update_rate_ = config["update_rate"].as<double>();
  }

  model_->GetBody("base")->physics_body_->SetLinearVelocity(b2Vec2(1000, 0));
  ROS_INFO_NAMED("Bumper Plugin", "Initialized");
}

void Bumper::BeforePhysicsStep(const Timekeeper &timekeeper) {
  model_->GetBody("base")->physics_body_->SetAngularVelocity(3);
  printf("************** Step ***************\n");
}

void Bumper::AfterPhysicsStep(const Timekeeper &timekeeper) {
  std::map<b2Contact *, ContactState>::iterator it;

  for (it = contacts_state_.begin(); it != contacts_state_.end(); it++) {
    b2Contact *c = it->first;
    ContactState *s = &it->second;
    flatland_msgs::ContactState contact_state_msg;
    contact_state_msg.entity = s->entity->name_;

    b2Manifold *m = c->GetManifold();

    for (int i = 0; i < m->pointCount; i++) {
      double ave_normal_impulse = s->sum_normal_impulses[i] / s->num_count;
      double ave_tangential_impulse =
          s->sum_tangential_impulses[i] / s->num_count;
      double ave_normal_force = ave_normal_impulse / timekeeper.GetStepSize();
      double ave_tangential_force =
          ave_tangential_impulse / timekeeper.GetStepSize();
      
    }
  }
}

void Bumper::BeginContact(b2Contact *contact) {
  Entity *entity;
  b2Fixture *fixture_A, *fixture_B;
  if (!FilterContact(contact, entity, fixture_A, fixture_B)) return;

  if (!contacts_state_.count(contact)) {
    contacts_state_[contact] = ContactState();
    contacts_state_[contact].entity = entity;
  }

  printf("BeginContact %p\n", contact);
}

void Bumper::EndContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  if (contacts_state_.count(contact)) {
    contacts_state_.erase(contact);
  } else {
    ROS_ERROR_NAMED("Bumper Plugin", "Unkown Box2D contact");
  }
}

void Bumper::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
  if (!FilterContact(contact)) return;

  ContactState *state;
  if (contacts_state_.count(contact)) {
    state = &contacts_state_[contact];
  } else {
    ROS_ERROR_NAMED("Bumper Plugin", "Unkown Box2D contact");
  }

  b2WorldManifold m;
  contact->GetWorldManifold(&m);

  state->num_count++;
  state->sum_normal_impulses[0] += impulse->normalImpulses[0];
  state->sum_normal_impulses[1] += impulse->normalImpulses[1];
  state->sum_tangential_impulses[0] += impulse->tangentImpulses[0];
  state->sum_tangential_impulses[1] += impulse->tangentImpulses[1];

  state->points[0] = m.points[0];
  state->points[1] = m.points[1];

  state->normal = m.normal;

  printf("PostSolve %p   [%f, %f], ((%f, %f), (%f, %f)), (%f, %f)\n", contact,
         impulse->normalImpulses[0], impulse->normalImpulses[1], m.points[0].x,
         m.points[0].y, m.points[1].x, m.points[1].y, m.normal.x, m.normal.y);
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Bumper, flatland_server::ModelPlugin)
