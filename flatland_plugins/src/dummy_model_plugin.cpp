/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  dummy_model_plugin.cpp
 * @brief   Dummy model plugin
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

#include <flatland_plugins/dummy_model_plugin.h>
#include <flatland_server/model_plugin.h>
#include <pluginlib/class_list_macros.h>

using namespace flatland_server;

namespace flatland_plugins {

DummyModelPlugin::DummyModelPlugin() {
  ClearTestingVariables();
}

void DummyModelPlugin::ClearTestingVariables() {
  initialized = false;
  cfg = YAML::Node();
  timestep = -1;
  layer = nullptr;
  model = nullptr;
  fixture_A = nullptr;
  fixture_B = nullptr;
  contact = nullptr;
}

void DummyModelPlugin::OnInitialize(const YAML::Node &config) {
  cfg = config;
}

void DummyModelPlugin::BeforePhysicsStep(double timestep) {}

void DummyModelPlugin::AfterPhysicsStep(double timestep) {}

void DummyModelPlugin::BeginContactWithMap(Layer *layer,
                                           b2Fixture *layer_fixture,
                                           b2Fixture *this_fixture,
                                           b2Contact *contact) {}

void DummyModelPlugin::BeginContactWithModel(Model *model,
                                             b2Fixture *model_fixture,
                                             b2Fixture *this_fixture,
                                             b2Contact *contact) {}

void DummyModelPlugin::EndContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                                         b2Fixture *this_fixture,
                                         b2Contact *contact) {}

void DummyModelPlugin::EndContactWithModel(Model *model,
                                           b2Fixture *model_fixture,
                                           b2Fixture *this_fixture,
                                           b2Contact *contact) {}

void DummyModelPlugin::BeginContact(b2Contact *contact) {}

void DummyModelPlugin::EndContact(b2Contact *contact) {}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::DummyModelPlugin,
                       flatland_server::ModelPlugin)