/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 joint.h
 * @brief	 Defines Joint
 * @author   Chunshang Li
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

#ifndef FLATLAND_SERVER_JOINT_H
#define FLATLAND_SERVER_JOINT_H

#include <flatland_server/model.h>
#include <flatland_server/types.h>
#include <flatland_server/yaml_reader.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

class Model;

/**
 * This class defines a joint in the simulation world. It wraps around the Box2D
 * physics joints providing extra data and useful methods
 */
class Joint {
 public:
  Model *model_;            ///< Model the joint belongs to
  std::string name_;        ///< Name of the joint
  b2World *physics_world_;  ///< Box2D physics world
  Color color_;             ///< Color for visualization
  b2Joint *physics_joint_;  ///< Box2D physics joint

  /**
   * @brief Constructor for the joint
   * @param[in] physics_world Box2D physics world
   * @param[in] model Model the joint belongs to
   * @param[in] name Name of the joint
   * @param[in] color Color to visualize the joint
   * @param[in] joint_def Box2D joint definition
   */
  Joint(b2World *physics_world, Model *model, const std::string &name,
        const Color &color, const b2JointDef &joint_def);
  ~Joint();

  /// Disallow copying of joints, problematic for constructors and destructors
  Joint(const Joint &) = delete;
  Joint &operator=(const Joint &) = delete;

  /**
   * @brief logs the debugging information for the joint
   */
  void DebugOutput() const;

  /**
   * @return Pointer to the model that contains the joint
   */
  Model *GetModel();

  /**
   * @return Name of the the joint
   */
  const std::string &GetName() const;

  /**
   * @return Color of the joint for visualization
   */
  const Color &GetColor() const;

  /**
   * @return Color of the joint for visualization
   */
  void SetColor(const Color &color);

  /**
   * @return Get pointer to the Box2D physics joint
   */
  b2Joint *GetPhysicsJoint();

  /**
   * @return Get pointer of the Box2D physics world
   */
  b2World *GetphysicsWorld();

  /**
   * @brief Creates a joint for the given params, throws exceptions upon failure
   * @param[in] physics_world Box2D physics world
   * @param[in] model Model the joint belongs to
   * @param[in] joint_reader YAML reader for node that contains joint
   * information
   * @return A new joint as defined by the input data
   */
  static Joint *MakeJoint(b2World *physics_world, Model *model,
                          YamlReader &joint_reader);

  /**
   * @brief Creates a revolute joint for the given params, throws exceptions
   * upon failure
   * @param[in] physics_world Box2D physics world
   * @param[in] model Model the joint belongs to
   * @param[in] joint_reader YAML reader for node that contains joint
   * information
   * @param[in] name Name of the joint
   * @param[in] color Color to visualize the joint
   * @param[in] body_A pointer to the first body
   * @param[in] anchor_B anchor point on the first body
   * @param[in] body_B pointer to the second body
   * @param[in] anchor_B anchor point on the second body
   * @param[in] collide_connected Should two bodies connected by this joint
   * collide
   * @return A new revolute joint as defined by the input data
   */
  static Joint *MakeRevoluteJoint(b2World *physics_world, Model *model,
                                  YamlReader &joint_reader,
                                  const std::string &name, const Color &color,
                                  b2Body *body_A, b2Vec2 anchor_A,
                                  b2Body *body_B, b2Vec2 anchor_B,
                                  bool collide_connected);

  /**
   * @brief Creates a weld joint for the given params, throws exceptions upon
   * failure
   * @param[in] physics_world Box2D physics world
   * @param[in] model Model the joint belongs to
   * @param[in] joint_reader YAML reader for node that contains joint
   * information
   * @param[in] name Name of the joint
   * @param[in] color Color to visualize the joint
   * @param[in] body_A pointer to the first body
   * @param[in] anchor_B anchor point on the first body
   * @param[in] body_B pointer to the second body
   * @param[in] anchor_B anchor point on the second body
   * @param[in] collide_connected Should two bodies connected by this joint
   * collide
   * @return A new weld joint as defined by the input data
   */
  static Joint *MakeWeldJoint(b2World *physics_world, Model *model,
                              YamlReader &joint_reader, const std::string &name,
                              const Color &color, b2Body *body_A,
                              b2Vec2 anchor_A, b2Body *body_B, b2Vec2 anchor_B,
                              bool collide_connected);
};
};      // namespace flatland_server
#endif  // FLATLAND_MODEL_JOINT_H
