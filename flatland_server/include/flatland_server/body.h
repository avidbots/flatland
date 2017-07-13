/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 body.h
 * @brief	 Defines Body
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

#ifndef FLATLAND_BODY_H
#define FLATLAND_BODY_H

#include <flatland_server/entity.h>

namespace flatland_server {

/**
 * This class defines a body in the simulation. It wraps around the Box2D
 * physics body providing extra data and useful methods
 */
class Body {
 public:
  Entity *entity_;               ///< The entity the body belongs to
  std::string name_;             ///< name of the body, unique within a model
  b2Body *physics_body_;         ///< Box2D physics body
  std::array<double, 4> color_;  ///< color, for visualization

  /**
   * @brief constructor for body, takes in all the required parameters
   * @param[in] physics_world Box2D physics world
   * @param[in] entity Entity the body is tied to
   * @param[in] name Name for the body
   * @param[in] color Color in r, g, b, a
   * @param[in] origin Origin of body's coordinate system in x, y, yaw
   * @param[in] body_type Box2D body type either dynamic, kinematic, or static
   * @param[in] linear_damping Box2D body linear damping
   * @param[in] angular_damping Box2D body angular damping
   */
  Body(b2World *physics_world, Entity *entity, const std::string &name,
       const std::array<double, 4> &color, const std::array<double, 3> &origin,
       b2BodyType body_type, double linear_damping, double angular_damping);

  /**
   * Destructor for the body
   */
  virtual ~Body();

  /**
   * Prevent copying since it is creates complications with constructing
   * and destructing bodies
   */
  Body(const Body &) = delete;
  Body &operator=(const Body &) = delete;
};
};      // namespace flatland_server
#endif  // FLATLAND_MODEL_BODY_H
