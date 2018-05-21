/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 entity.h
 * @brief	 Defines flatland Entity
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

#ifndef FLATLAND_SERVER_ENTITY_H
#define FLATLAND_SERVER_ENTITY_H

#include <Box2D/Box2D.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

/**
 * This class defines a entity in the simulation world. It provides a class
 * for high level physical things in the world to inherit from (layers and
 * models)
 */
class Entity {
 public:
  /// Defines the type of entity
  enum EntityType { LAYER, MODEL };

  b2World *physics_world_;  ///< Box2D physics world
  std::string name_;        ///< name of the entity

  /**
   * @brief Constructor for the entity
   * @param[in] physics_world Box2D physics_world
   * @param[in] name name of the entity
   */
  Entity(b2World *physics_world, const std::string &name);
  virtual ~Entity() = default;

  /**
   * @return name of the entity
   */
  const std::string &GetName() const;

  /**
   * @brief Get Box2D physics world
   * @return Pointer to Box2D physics world, use this to call Box2D world
   * methods
   */
  b2World *GetPhysicsWorld();

  /// This class should be non-copyable. This will cause the destructor to be
  /// called twice for a given b2Body
  Entity(const Entity &) = delete;
  Entity &operator=(const Entity &) = delete;

  /**
   * @brief Get the type of entity, subclasses must override
   * @return the type of entity
   */
  virtual EntityType Type() const = 0;

  /**
   * @brief Visualize the entity
   */
  virtual void DebugVisualize() const = 0;

  /**
   * @brief Print debug message for the entity
   */
  virtual void DebugOutput() const = 0;
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_ENTITY_H
