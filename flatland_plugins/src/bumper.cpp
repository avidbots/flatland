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

#include <flatland_msgs/Collision.h>
#include <flatland_msgs/Collisions.h>
#include <flatland_plugins/bumper.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string/join.hpp>

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
  YamlReader reader(config);

  // defaults
  world_frame_id_ = reader.Get<std::string>("world_frame_id", "map");
  topic_name_ = reader.Get<std::string>("topic", "collisions");
  publish_all_collisions_ = reader.Get<bool>("publish_all_collisions", true);
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  std::vector<std::string> excluded_body_names =
      reader.GetList<std::string>("exclude", {}, -1, -1);

  reader.EnsureAccessedAllKeys();

  for (unsigned int i = 0; i < excluded_body_names.size(); i++) {
    Body *body = GetModel()->GetBody(excluded_body_names[i]);

    if (body == nullptr) {
      throw YAMLException("Body with name \"" + excluded_body_names[i] +
                          "\" does not exist");
    } else {
      excluded_bodies_.push_back(body);
    }
  }

  update_timer_.SetRate(update_rate_);
  collisions_publisher_ =
      nh_.advertise<flatland_msgs::Collisions>(topic_name_, 1);

  ROS_DEBUG_NAMED("Bumper",
                  "Initialized with params: topic(%s) world_frame_id(%s) "
                  "publish_all_collisions(%d) update_rate(%f) exclude({%s})",
                  topic_name_.c_str(), world_frame_id_.c_str(),
                  publish_all_collisions_, update_rate_,
                  boost::algorithm::join(excluded_body_names, ",").c_str());
}

void Bumper::BeforePhysicsStep(const Timekeeper &timekeeper) {
  std::map<b2Contact *, ContactState>::iterator it;

  // Clear the forces at the begining of every physics step since brand
  // new collision resolutions are being calculated by Box2D each time step
  for (it = contact_states_.begin(); it != contact_states_.end(); it++) {
    it->second.Reset();
  }
}

void Bumper::AfterPhysicsStep(const Timekeeper &timekeeper) {
  // The expected behaviour is to always publish non-empty collisions unless
  // publish_all_collisions set to false. The publishing of empty collision
  // manages the publishing rate of empty collisions when
  // publish_all_collisions is true, or it manages the publishing rate all
  // empty and non-empty collisions when publish_all_collisions_ is false
  if (!publish_all_collisions_ || contact_states_.size() <= 0) {
    if (!update_timer_.CheckUpdate(timekeeper)) {
      return;
    }
  }

  std::map<b2Contact *, ContactState>::iterator it;

  flatland_msgs::Collisions collisions;
  collisions.header.frame_id = world_frame_id_;
  collisions.header.stamp = timekeeper.GetSimTime();

  // loop through all collisions in our record and publish
  for (it = contact_states_.begin(); it != contact_states_.end(); it++) {
    b2Contact *c = it->first;
    ContactState *s = &it->second;
    flatland_msgs::Collision collision;
    collision.entity_A = GetModel()->GetName();
    collision.entity_B = s->entity_B->name_;

    collision.body_A = s->body_A->name_;
    collision.body_B = s->body_B->name_;

    // If there was no post solve called, which means that the collision
    // probably involves a Box2D sensor, therefore there are no contact points,
    if (s->num_count > 0) {
      b2Manifold *m = c->GetManifold();

      // go through each collision point
      for (int i = 0; i < m->pointCount; i++) {
        // calculate average impulse during each time step, the impulse are
        // converted to the applied force by dividing the step size
        double ave_normal_impulse = s->sum_normal_impulses[i] / s->num_count;
        double ave_tangential_impulse =
            s->sum_tangential_impulses[i] / s->num_count;
        double ave_normal_force = ave_normal_impulse / timekeeper.GetStepSize();
        double ave_tangential_force =
            ave_tangential_impulse / timekeeper.GetStepSize();

        // Calculate the absolute magnitude of forces, forces are not provided
        // in vector form because the forces are obtained from averaging
        // Box2D impulses which are very inaccurate making them completely
        // useless for anything other than ball parking the impact strength
        double force_abs = sqrt(ave_normal_force * ave_normal_force +
                                ave_tangential_force * ave_tangential_force);

        collision.magnitude_forces.push_back(force_abs);
        flatland_msgs::Vector2 point;
        flatland_msgs::Vector2 normal;
        point.x = s->points[i].x;
        point.y = s->points[i].y;
        normal.x = s->normal.x;
        normal.y = s->normal.y;
        collision.contact_positions.push_back(point);
        collision.contact_normals.push_back(normal);
      }
    }

    collisions.collisions.push_back(collision);
  }

  collisions_publisher_.publish(collisions);
}

void Bumper::BeginContact(b2Contact *contact) {
  Entity *other_entity;
  b2Fixture *this_fixture, *other_fixture;
  if (!FilterContact(contact, other_entity, this_fixture, other_fixture)) {
    return;
  }

  // If this is a new contact, add it to the records of alive contacts
  if (!contact_states_.count(contact)) {
    Body *collision_body =
        static_cast<Body *>(this_fixture->GetBody()->GetUserData());

    bool ignore = false;

    // check that the body is not in the ignore list
    for (unsigned int j = 0; j < excluded_bodies_.size(); j++) {
      if (excluded_bodies_[j] == collision_body) {
        ignore = true;
        break;
      }
    }

    // add the body to the record of active contacts
    if (!ignore) {
      contact_states_[contact] = ContactState();
      ContactState *c = &contact_states_[contact];
      c->entity_B = other_entity;
      c->body_B = static_cast<Body *>(other_fixture->GetBody()->GetUserData());
      c->body_A = collision_body;

      // by convention, Box2D normal goes from fixture A to fixture B, the
      // sign is used to correct cases when our model isn't fixture A, so that
      // normal always points from this fixture to other_fixture
      if (contact->GetFixtureA() == this_fixture) {
        c->normal_sign = 1;
      } else {
        c->normal_sign = -1;
      }
    }
  }
}

void Bumper::EndContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  // The contact ended, remove it from the list of contacts
  if (contact_states_.count(contact)) {
    contact_states_.erase(contact);
  } else {
    // contact is ignored
    return;
  }
}

void Bumper::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
  if (!FilterContact(contact)) return;

  ContactState *state;
  if (contact_states_.count(contact)) {
    state = &contact_states_[contact];
  } else {
    // contact is ignored
    return;
  }

  // post solve can be called multiple times per time step due to Box2D's
  // continuous collision detectopm where Box2D "substeps" in the solver. Each
  // substep returns the impulse applied to the body at that substep. We cannot
  // obtain the step size of these sub steps, so we just assumed that the
  // substeps occurred over evenly subdivided intervals (which is probably
  // false), and average all the received impulses later on. The points of
  // collision and normals also change slightly at each substep, we simply
  // always use ones from the most recent post solve. These results should
  // only be used to provide a ball park feel of impact strength

  b2WorldManifold m;
  contact->GetWorldManifold(&m);

  state->num_count++;

  // We take data from both contact points even though there might only be
  // one point of contact. We sum everything here, and only take the valid
  // ones later
  state->sum_normal_impulses[0] += impulse->normalImpulses[0];
  state->sum_normal_impulses[1] += impulse->normalImpulses[1];
  state->sum_tangential_impulses[0] += impulse->tangentImpulses[0];
  state->sum_tangential_impulses[1] += impulse->tangentImpulses[1];

  state->points[0] = m.points[0];
  state->points[1] = m.points[1];

  state->normal = m.normal;
  state->normal *= state->normal_sign;
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Bumper, flatland_server::ModelPlugin)
