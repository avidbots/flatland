/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	tween.h
 * @brief   Tween plugin
 * @author  Joseph Duchesne
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
#include <flatland_plugins/update_timer.h>
#include <flatland_server/model_plugin.h>
#include <flatland_server/timekeeper.h>
#include <flatland_server/types.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "tweeny.h"

#ifndef FLATLAND_PLUGINS_TWEEN_H
#define FLATLAND_PLUGINS_TWEEN_H

using namespace flatland_server;

namespace flatland_plugins {

class Tween : public flatland_server::ModelPlugin {
 public:
  Body* body_;
  Pose start_;
  Pose delta_;
  float duration_;
  tweeny::tween<double, double, double> tween_;

  enum class ModeType_ { YOYO, LOOP, ONCE };
  ModeType_ mode_;
  static const std::map<std::string, Tween::ModeType_> mode_strings_;

  enum class EasingType_ {
    linear,
    quadraticIn,
    quadraticOut,
    quadraticInOut,
    cubicIn,
    cubicOut,
    cubicInOut,
    quarticIn,
    quarticOut,
    quarticInOut,
    quinticIn,
    quinticOut,
    quinticInOut,
    // sinuisodal,
    exponentialIn,
    exponentialOut,
    exponentialInOut,
    circularIn,
    circularOut,
    circularInOut,
    backIn,
    backOut,
    backInOut,
    elasticIn,
    elasticOut,
    elasticInOut,
    bounceIn,
    bounceOut,
    bounceInOut
  };
  static const std::map<std::string, Tween::EasingType_> easing_strings_;

  /**
   * @name          OnInitialize
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void OnInitialize(const YAML::Node& config) override;
  /**
   * @name          BeforePhysicsStep
   * @brief         override the BeforePhysicsStep method
   * @param[in]     config The plugin YAML node
   */
  void BeforePhysicsStep(const Timekeeper& timekeeper) override;
};

const std::map<std::string, Tween::ModeType_> Tween::mode_strings_ = {
    {"yoyo", Tween::ModeType_::YOYO},
    {"loop", Tween::ModeType_::LOOP},
    {"once", Tween::ModeType_::ONCE}};

const std::map<std::string, Tween::EasingType_> Tween::easing_strings_ = {
    {"linear", Tween::EasingType_::linear},
    {"quadraticIn", Tween::EasingType_::quadraticIn},
    {"quadraticOut", Tween::EasingType_::quadraticOut},
    {"quadraticInOut", Tween::EasingType_::quadraticInOut},
    {"cubicIn", Tween::EasingType_::cubicIn},
    {"cubicOut", Tween::EasingType_::cubicOut},
    {"cubicInOut", Tween::EasingType_::cubicInOut},
    {"quarticIn", Tween::EasingType_::quarticIn},
    {"quarticOut", Tween::EasingType_::quarticOut},
    {"quarticInOut", Tween::EasingType_::quarticInOut},
    {"quinticIn", Tween::EasingType_::quinticIn},
    {"quinticOut", Tween::EasingType_::quinticOut},
    {"quinticInOut", Tween::EasingType_::quinticInOut},
    // { "sinuisodal", Tween::EasingType_::sinuisodal },
    {"exponentialIn", Tween::EasingType_::exponentialIn},
    {"exponentialOut", Tween::EasingType_::exponentialOut},
    {"exponentialInOut", Tween::EasingType_::exponentialInOut},
    {"circularIn", Tween::EasingType_::circularIn},
    {"circularOut", Tween::EasingType_::circularOut},
    {"circularInOut", Tween::EasingType_::circularInOut},
    {"backIn", Tween::EasingType_::backIn},
    {"backOut", Tween::EasingType_::backOut},
    {"backInOut", Tween::EasingType_::backInOut},
    {"elasticIn", Tween::EasingType_::elasticIn},
    {"elasticOut", Tween::EasingType_::elasticOut},
    {"elasticInOut", Tween::EasingType_::elasticInOut},
    {"bounceIn", Tween::EasingType_::bounceIn},
    {"bounceOut", Tween::EasingType_::bounceOut},
    {"bounceInOut", Tween::EasingType_::bounceInOut}};
};

#endif