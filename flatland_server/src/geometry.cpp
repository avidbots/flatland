/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  geometry.cpp
 * @brief     Geometry functions
 * @author    Joseph Duchesne
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

#include "flatland_server/geometry.h"
#include <Box2D/Box2D.h>
#include <cmath>

namespace flatland_server {

/**
 * @brief Return a RotateTranslate given translation and rotation
 *
 * @param dx X translation
 * @param dy Y translation
 * @param a  rotation (radians)
 *
 * @return THe RotateTranslate object
 */
RotateTranslate Geometry::CreateTransform(double dx, double dy, double a) {
  RotateTranslate out = {dx, dy, cosf(a), sinf(a)};
  return out;
}

/**
 * @param Transform a b2Vec2 copy by a RotateTranslate object
 *
 * @param in The input vector
 * @param rt The transformation
 *
 * @return
 */
b2Vec2 Geometry::Transform(const b2Vec2& in, const RotateTranslate& rt) {
  b2Vec2 out;
  out.x = in.x * rt.cos - in.y * rt.sin + rt.dx;
  out.y = in.x * rt.sin + in.y * rt.cos + rt.dy;
  return out;
}

b2Vec2 Geometry::InverseTransform(const b2Vec2& in, const RotateTranslate& rt) {
  b2Vec2 out;
  out.x = (in.x - rt.dx) * rt.cos + (in.y - rt.dy) * rt.sin;
  out.y = -(in.x - rt.dx) * rt.sin + (in.y - rt.dy) * rt.cos;
  return out;
}
};
