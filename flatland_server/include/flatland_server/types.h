/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 types.h
 * @brief	 Defines common types in flatland
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

#include <Box2D/Box2D.h>
#include <geometry_msgs/Pose2D.h>

#ifndef FLATLAND_SERVER_TYPES_H
#define FLATLAND_SERVER_TYPES_H

namespace flatland_server {

struct Vec2 {
  double x;
  double y;

  Vec2(double x, double y) {
    this->x = x;
    this->y = y;
  }

  Vec2() : x(0), y(0) {}

  b2Vec2 Box2D() const { return b2Vec2(x, y); }
};

struct LineSegment {
  Vec2 start;
  Vec2 end;

  LineSegment(const Vec2 &start, const Vec2 &end) {
    this->start = start;
    this->end = end;
  }

  LineSegment() {
    this->start = Vec2(0, 0);
    this->end = Vec2(0, 0);
  }
};

struct Pose {
  double x;
  double y;
  double theta;  ///< theta

  Pose(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }

  Pose(const std::array<double, 3> &p) {
    this->x = p[0];
    this->y = p[1];
    this->theta = p[3];
  }

  Pose() : x(0), y(0), theta(0) {}

  bool operator==(const Pose &p) const {
    return x == p.x && y == p.y && theta == p.theta;
  }

  bool operator!=(const Pose &p) const { return !operator==(p); }
};

struct Color {
  double r, g, b, a;

  Color() : r(0), g(0), b(0), a(0) {}

  Color(double r, double g, double b, double a) {
    this->r = r;
    this->g = g;
    this->b = b;
    this->a = a;
  }

  Color(const std::array<double, 4> &c) {
    this->r = c[0];
    this->g = c[1];
    this->b = c[2];
    this->a = c[3];
  }

  bool operator==(const Color &c) const {
    return r == c.r && g == c.g && b == c.b && a == c.a;
  }

  bool operator!=(const Color &c) const { return !operator==(c); }
};
}

#endif
