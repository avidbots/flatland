/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	model_plugin.h
 * @brief	Interface for ModelPlugin pluginlib plugins
 * @author Joseph Duchesne
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

#ifndef FLATLAND_SERVER_MODEL_PLUGIN_H
#define FLATLAND_SERVER_MODEL_PLUGIN_H

#include <Box2D/Box2D.h>
#include <flatland_server/layer.h>
#include <flatland_server/model.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

/**
 * This class defines a model plugin. All implemented model plugins will inherit
 * from it A model plugin is a plugin that is directly tied to a single model in
 * the world
 */
class ModelPlugin {
 public:
  std::string type_;    ///< type of the plugin
  std::string name_;    ///< name of the plugin
  ros::NodeHandle nh_;  ///< ROS node handle
  Model *model_;        ///< model this plugin is tied to

  /**
   * @brief The method to initialize the ModelPlugin, required since Pluginlib
   * require the class to have a default constructor
   * @param[in] type Type of the plugin
   * @param[in] name Name of the plugin
   * @param[in] model The model associated with this model plugin
   * @param[in] config The plugin YAML node
   */
  void Initialize(const std::string &type, const std::string &name,
                  Model *model, const YAML::Node &config);

  /**
   * @brief The method for the particular model plugin to override and provide
   * its own initialization
   * @param[in] config The plugin YAML node
   */
  virtual void OnInitialize(const YAML::Node &config) = 0;

  /**
   * @brief This method is called before the Box2D physics step
   * @param[in] timestep how much the physics time will increment
   */
  virtual void BeforePhysicsStep(double timestep) {}

  /**
   * @brief This method is called after the Box2D physics step
   * @param[in] timestep how much the physics time have incremented
   */
  virtual void AfterPhysicsStep(double timestep) {}

  /**
   * @brief This method is called when the model starts to collide with the map
   * (layer)
   * @param[in] layer The layer that collided with the model
   * @param[in] layer_fixture The fixture in the collided layer
   * @param[in] this_fixture The fixture in the model that collided with the map
   * @param[in] contact Box2D contact contain all relevant contact data
   */
  virtual void BeginContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                                   b2Fixture *this_fixture,
                                   b2Contact *contact) {}

  /**
   * @brief This method is called when this model starts to collided with
   * the other model
   * @param[in] model The other model that this model collided with
   * @param[in] model_fixture The fixture in the other model
   * @param[in] this_fixture The fixture in this model
   * @param[in] contact Box2D contact contain all relevant contact data
   */
  virtual void BeginContactWithModel(Model *model, b2Fixture *model_fixture,
                                     b2Fixture *this_fixture,
                                     b2Contact *contact) {}

  /**
   * @brief This method is called when the model stopped collide with another
   * model, i.e. the model and layer are no longer contacting
   * @param[in] layer The layer that collided with the model
   * @param[in] layer_fixture The fixture in the collided layer
   * @param[in] this_fixture The fixture in the model that collided with the map
   * @param[in] contact Box2D contact contain all relevant contact data
   */
  virtual void EndContactWithMap(Layer *layer, b2Fixture *layer_fixture,
                                 b2Fixture *this_fixture, b2Contact *contact) {}

  /**
   * @brief This method is called when this model stopped colliding with the
   * other model, i.e. the models are no longer contacting
   * @param[in] model The other model that this model collided with
   * @param[in] model_fixture The fixture in the other model
   * @param[in] this_fixture The fixture in this model
   * @param[in] contact Box2D contact contain all relevant contact data
   */
  virtual void EndContactWithModel(Model *model, b2Fixture *model_fixture,
                                   b2Fixture *this_fixture,
                                   b2Contact *contact) {}

  /**
   * @brief A method that is called for all Box2D begin contacts
   */
  virtual void BeginContact(b2Contact *contact) {}

  /**
   * @brief A method that is called for all Box2D end contacts
   */
  virtual void EndContact(b2Contact *contact) {}

  /**
   * @brief Model plugin destructor
   */
  virtual ~ModelPlugin() = default;

 protected:
  /**
   * @brief Model plugin default constructor
   */
  ModelPlugin() = default;
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_MODEL_PLUGIN_H
