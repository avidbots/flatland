/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name  flatland_plugin.h
 * @brief Interface for Flatland pluginlib plugins
 * @author Yi Ren
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

#ifndef FLATLAND_SERVER_FLATLAND_PLUGIN_H
#define FLATLAND_SERVER_FLATLAND_PLUGIN_H

#include <Box2D/Box2D.h>
#include <flatland_server/timekeeper.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <string>

namespace flatland_server {
class FlatlandPlugin {
 public:
  enum class PluginType { Invalid, Model, World };  // Different plugin Types
  std::string type_;                                ///< type of the plugin
  std::string name_;                                ///< name of the plugin
  ros::NodeHandle nh_;                              // ROS node handle
  PluginType plugin_type_;

  /*
  * @brief Get PluginType
  */
  const PluginType Type() { return plugin_type_; }

  /**
  * @brief Get plugin name
  */
  const std::string &GetName() const { return name_; }

  /**
  * @brief Get type of plugin
  */
  const std::string &GetType() const { return type_; }

  /**
 * @brief The method for the particular model plugin to override and provide
 * its own initialization
 * @param[in] config The plugin YAML node
 */
  virtual void OnInitialize(const YAML::Node &config) = 0;

  /**
   * @brief This method is called before the Box2D physics step
   * @param[in] timekeeper provide time related information
   */
  virtual void BeforePhysicsStep(const Timekeeper &timekeeper) {}

  /**
   * @brief This method is called after the Box2D physics step
   * @param[in] timekeeper provide time related information
   */
  virtual void AfterPhysicsStep(const Timekeeper &timekeeper) {}

  /**
   * @brief A method that is called for all Box2D begin contacts
   * @param[in] contact Box2D contact
   */
  virtual void BeginContact(b2Contact *contact) {}

  /**
   * @brief A method that is called for all Box2D end contacts
   * @param[in] contact Box2D contact
   */
  virtual void EndContact(b2Contact *contact) {}

  /**
   * @brief A method that is called for Box2D presolve
   * @param[in] contact Box2D contact
   * @param[in] oldManifold Manifold from the previous iteration
   */
  virtual void PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {}

  /**
   * @brief A method that is called for Box2D postsolve
   * @param[in] contact Box2D contact
   * @param[in] impulse Impulse from the collision resolution
   */
  virtual void PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {}

  /**
   * @brief Flatland plugin destructor
   */
  virtual ~FlatlandPlugin() = default;
};
};  // namespace

#endif  // FLATLAND_SERVER_FLATLAND_PLUGIN_H