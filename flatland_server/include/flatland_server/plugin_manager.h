/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 plugin_manager.h
 * @brief	 Definition for plugin manager
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

#ifndef FLATLAND_PLUGIN_MANAGER_H
#define FLATLAND_PLUGIN_MANAGER_H

#include <Box2D/Box2D.h>
#include <flatland_server/model.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/world_plugin.h>
#include <flatland_server/yaml_reader.h>
#include <pluginlib/class_loader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

// forward declaration
class World;

class PluginManager {
 public:
  std::vector<boost::shared_ptr<ModelPlugin>> model_plugins_;
  pluginlib::ClassLoader<flatland_server::ModelPlugin> *model_plugin_loader_;

  std::vector<boost::shared_ptr<WorldPlugin>> world_plugins_;
  pluginlib::ClassLoader<flatland_server::WorldPlugin> *world_plugin_loader_;
  /**
   * @brief Plugin manager constructor
   */
  PluginManager();

  /**
   * @brief Plugin manager destructor
   */
  ~PluginManager();

  /**
   * @brief This method is called before the Box2D physics step
   * @param[in] timekeeper provide time related information
   */
  void BeforePhysicsStep(const Timekeeper &timekeeper);

  /**
   * @brief This method is called after the Box2D physics step
   * @param[in] timekeeper provide time related information
   */
  void AfterPhysicsStep(const Timekeeper &timekeeper);

  /**
   * @brief This method removes all model plugins associated with a given mode
   * @param[in] The model plugins is associated to
   */
  void DeleteModelPlugin(Model *model);

  /**
   * @brief Load model plugins
   * @param[in] model The model that this plugin is tied to
   * @param[in] plugin_reader The YAML reader with node containing the plugin
   * parameter
   */
  void LoadModelPlugin(Model *model, YamlReader &plugin_reader);

  /*
   * @brief load world plugins
   * @param[in] world, the world that thsi plugin is tied to
   * @param[in] plugin_reader, the YAML reader with node containing the plugin
   * @param[in] world_config, the yaml reader of world.yaml
  */
  void LoadWorldPlugin(World *world, YamlReader &plugin_reader,
                       YamlReader &world_config);

  /**
   * @brief Method called for a box2D begin contact
   * @param[in] contact Box2D contact information
   */
  void BeginContact(b2Contact *contact);

  /**
   * @brief Method called for a box2D end contact
   * @param[in] contact Box2D contact information
   */
  void EndContact(b2Contact *contact);

  /**
   * @brief Method called for Box2D presolve
   * @param[in] contact Box2D contact information
   * @param[in] oldManifold The manifold from the previous timestep
   */
  void PreSolve(b2Contact *contact, const b2Manifold *oldManifold);

  /**
   * @brief Method called for Box2D Postsolve
   * @param[in] contact Box2D contact information
   * @param[in] impulse The calculated impulse from the collision resolute
   */
  void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse);
};
};      // namespace flatland_server
#endif  // FLATLAND_PLUGIN_MANAGER_H
