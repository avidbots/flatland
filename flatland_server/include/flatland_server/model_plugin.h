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
#include <flatland_server/flatland_plugin.h>
#include <flatland_server/model.h>
#include <flatland_server/timekeeper.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

/**
 * This class defines a model plugin. All implemented model plugins will inherit
 * from it A model plugin is a plugin that is directly tied to a single model in
 * the world
 */
class ModelPlugin : public FlatlandPlugin {
 private:
  Model *model_;  ///< model this plugin is tied to

 public:
  ros::NodeHandle nh_;  ///< ROS node handle

  /**
   * @brief Get model
   */
  Model *GetModel();

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
   * @brief Helper function check if this model is part of the contact, and
   * extracts all the useful information
   * @param[in] contact Box2D contact
   * @param[out] entity The entity that collided with this model
   * @param[out] this_fixture The fixture from this model involved in the
   * collision
   * @param[out] other_fixture The fixture from the other entity involved in the
   * collision
   * @return True or false depending on if this model is involved. If false
   * is returned, none of the entity, this_fixture, other_fixture pointers will
   * be populated
   */
  bool FilterContact(b2Contact *contact, Entity *&entity,
                     b2Fixture *&this_fixture, b2Fixture *&other_fixture);

  /**
   * @brief Helper function check if this model is part of the contact
   * @param[in] contact Box2D contact
   * @return True or false depending on if this model is involved
   */
  bool FilterContact(b2Contact *contact);

 protected:
  /**
   * @brief Model plugin default constructor
   */
  ModelPlugin() = default;
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_MODEL_PLUGIN_H
