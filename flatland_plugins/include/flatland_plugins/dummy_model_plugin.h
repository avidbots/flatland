/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  dummy_model_plugin.h
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

#include <flatland_server/model_plugin.h>
#include <flatland_server/model.h>
#include <flatland_server/layer.h>
#include <Box2D/Box2D.h>
#include <yaml-cpp/yaml.h>

#ifndef FLATLAND_PLUGINS_DUMMY_MODEL_PLUGIN_H
#define FLATLAND_PLUGINS_DUMMY_MODEL_PLUGIN_H

using namespace flatland_server;

namespace flatland_plugins {

class DummyModelPlugin : public flatland_server::ModelPlugin {
 public:
  // variables used for testing
  bool initialized;
  YAML::Node cfg;
  double timestep;
  Layer *layer;
  Model *model;
  b2Fixture *fixture_A;
  b2Fixture *fixture_B;
  b2Contact *contact;

  DummyModelPlugin();

  virtual void OnInitialize(const YAML::Node &config) override;

  virtual void BeforePhysicsStep(double timestep) override;

  virtual void AfterPhysicsStep(double timestep) override;

  virtual void BeginContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                                   b2Fixture *this_fixture,
                                   b2Contact *contact) override;

  virtual void BeginContactWithModel(Model *model, b2Fixture *model_fixture,
                                     b2Fixture *this_fixture,
                                     b2Contact *contact) override;

  virtual void EndContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                                 b2Fixture *this_fixture,
                                 b2Contact *contact) override;

  virtual void EndContactWithModel(Model *model, b2Fixture *model_fixture,
                                   b2Fixture *this_fixture,
                                   b2Contact *contact) override;

  virtual void BeginContact(b2Contact *contact) override;

  virtual void EndContact(b2Contact *contact) override;

  void ClearTestingVariables();
};
};

#endif