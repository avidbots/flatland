/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	model_plugin.cpp
 * @brief	Implementation for ModelPlugin pluginlib plugins
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

#include <flatland_server/model_plugin.h>

namespace flatland_server {

Model *ModelPlugin::GetModel() { return model_; }

void ModelPlugin::Initialize(const std::string &type, const std::string &name,
                             Model *model, const YAML::Node &config) {
  type_ = type;
  name_ = name;
  model_ = model;
  plugin_type_ = PluginType::Model;
  // model_->namespace_ will only be used as tf_prefix
  // nh_ = ros::NodeHandle(model_->namespace_);
  nh_ = ros::NodeHandle();
  OnInitialize(config);
}

bool ModelPlugin::FilterContact(b2Contact *contact, Entity *&entity,
                                b2Fixture *&this_fixture,
                                b2Fixture *&other_fixture) {
  b2Fixture *f_A = contact->GetFixtureA();
  b2Fixture *f_B = contact->GetFixtureB();
  Body *b_A = static_cast<Body *>(f_A->GetBody()->GetUserData());
  Body *b_B = static_cast<Body *>(f_B->GetBody()->GetUserData());
  Entity *e_A = b_A->GetEntity();
  Entity *e_B = b_B->GetEntity();

  if (e_A == model_) {
    entity = e_B;
    this_fixture = f_A;
    other_fixture = f_B;
  } else if (e_B == model_) {
    entity = e_A;
    this_fixture = f_B;
    other_fixture = f_A;
  } else {
    return false;
  }
  return true;
}

bool ModelPlugin::FilterContact(b2Contact *contact) {
  b2Fixture *f1, *f2;
  Entity *e;
  return FilterContact(contact, e, f1, f2);
}

};  // namespace flatland_server
