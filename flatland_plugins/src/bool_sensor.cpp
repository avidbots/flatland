/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  bool_sensor.h
 * @brief   BoolSensor plugin
 * @author  Joseph Duchesne
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

#include <flatland_plugins/bool_sensor.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_list_macros.h>

using namespace flatland_server;

namespace flatland_plugins {

void BoolSensor::OnInitialize(const YAML::Node &config) {
  YamlReader reader(config);

  // defaults
  std::string topic_name = reader.Get<std::string>("topic", "bool_sensor");
  update_rate_ = reader.Get<double>("update_rate",
                                    std::numeric_limits<double>::infinity());

  // sensor defaults to the first model in the list
  if (GetModel()->bodies_.size() == 0) {
    throw YAMLException("You didn't provide any bodies for model" +
                        GetModel()->name_);
  }
  std::string body_name =
      reader.Get<std::string>("body", GetModel()->bodies_[0]->name_);

  reader.EnsureAccessedAllKeys();

  // Load the body pointer
  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name \"" + body_name + "\" does not exist");
  }

  // Set the update timer
  update_timer_.SetRate(update_rate_);

  // Init publisher
  publisher_ =
      nh_.advertise<std_msgs::Bool>(GetModel()->NameSpaceTopic(topic_name), 1);

  ROS_DEBUG_NAMED("BoolSensor",
                  "Initialized with params: topic(%s) body(%s) "
                  "update_rate(%f)",
                  topic_name.c_str(), body_name.c_str(), update_rate_);
}

void BoolSensor::AfterPhysicsStep(const Timekeeper &timekeeper) {
  // Publish the boolean timer at the desired update rate
  if (!update_timer_.CheckUpdate(timekeeper)) {
    return;
  }

  // This logic allows collisions that occur and resolve faster than the update
  // rate to result in at least one "true" publish
  std_msgs::Bool msg;
  if (hit_something_) {  // We hit something since the last publish
    msg.data = true;
    hit_something_ = false;  // Reset for next time
  } else {                   // Publish current state as usual
    if (collisions_ > 0) {
      msg.data = true;
    } else {
      msg.data = false;
    }
  }
  publisher_.publish(msg);
}

void BoolSensor::BeginContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  // Skip collisions with other fixtures on this body
  if (contact->GetFixtureA()->GetBody() == contact->GetFixtureB()->GetBody()) {
    return;
  }

  collisions_++;
  hit_something_ = true;
}

void BoolSensor::EndContact(b2Contact *contact) {
  if (!FilterContact(contact)) return;

  // Skip collisions with other fixtures on this body
  if (contact->GetFixtureA()->GetBody() == contact->GetFixtureB()->GetBody()) {
    return;
  }

  collisions_--;
}
}  // End flatland_plugins namespace

PLUGINLIB_EXPORT_CLASS(flatland_plugins::BoolSensor,
                       flatland_server::ModelPlugin)
