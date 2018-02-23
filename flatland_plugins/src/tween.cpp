/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	Tween.cpp
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
#include <flatland_plugins/tween.h>
#include <flatland_server/debug_visualization.h>
#include <flatland_server/model_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>

namespace flatland_plugins {

std::map<std::string, Tween::ModeType_> Tween::mode_strings_ = {
    {"yoyo", Tween::ModeType_::YOYO},
    {"loop", Tween::ModeType_::LOOP},
    {"once", Tween::ModeType_::ONCE},
    {"trigger", Tween::ModeType_::TRIGGER}};

std::map<std::string, Tween::EasingType_> Tween::easing_strings_ = {
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

void Tween::OnInitialize(const YAML::Node& config) {
  YamlReader reader(config);
  std::string body_name = reader.Get<std::string>("body");

  // reciprocal, loop, or oneshot
  std::string mode = reader.Get<std::string>("mode", "yoyo");
  duration_ = reader.Get<float>("duration", 1.0);

  delta_ = reader.GetPose("delta", Pose(0, 0, 0));

  // Boolean play pause topic
  std::string trigger_topic = reader.Get<std::string>("trigger_topic", "");
  if (trigger_topic != "") {
    trigger_sub_ =
        nh_.subscribe(trigger_topic, 1, &Tween::TriggerCallback, this);
  }

  body_ = GetModel()->GetBody(body_name);
  if (body_ == nullptr) {
    throw YAMLException("Body with name " + Q(body_name) + " does not exist");
  }
  start_ = Pose(body_->physics_body_->GetPosition().x,
                body_->physics_body_->GetPosition().y,
                body_->physics_body_->GetAngle());

  // Validate the mode selection
  if (!Tween::mode_strings_.count(mode)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  mode_ = Tween::mode_strings_.at(mode);

  tween_ = tweeny::from(0.0, 0.0, 0.0)
               .to(delta_.x, delta_.y, delta_.theta)
               .during((uint32)(duration_ * 1000.0));

  Tween::EasingType_ easing_type;
  std::string easing = reader.Get<std::string>("easing", "linear");
  if (!Tween::easing_strings_.count(easing)) {
    throw YAMLException("Mode " + mode + " does not exist");
  }
  easing_type = Tween::easing_strings_.at(easing);

  // This is clumsy but because tweeny used structs for each tweening rather
  // than subclasses
  // I believe that this is the best way to do this
  switch (easing_type) {
    case Tween::EasingType_::linear:
      tween_ = tween_.via(tweeny::easing::linear);
      break;
    case Tween::EasingType_::quadraticIn:
      tween_ = tween_.via(tweeny::easing::quadraticIn);
      break;
    case Tween::EasingType_::quadraticOut:
      tween_ = tween_.via(tweeny::easing::quadraticOut);
      break;
    case Tween::EasingType_::quadraticInOut:
      tween_ = tween_.via(tweeny::easing::quadraticInOut);
      break;
    case Tween::EasingType_::cubicIn:
      tween_ = tween_.via(tweeny::easing::cubicIn);
      break;
    case Tween::EasingType_::cubicOut:
      tween_ = tween_.via(tweeny::easing::cubicOut);
      break;
    case Tween::EasingType_::cubicInOut:
      tween_ = tween_.via(tweeny::easing::cubicInOut);
      break;
    case Tween::EasingType_::quarticIn:
      tween_ = tween_.via(tweeny::easing::quarticIn);
      break;
    case Tween::EasingType_::quarticOut:
      tween_ = tween_.via(tweeny::easing::quarticOut);
      break;
    case Tween::EasingType_::quarticInOut:
      tween_ = tween_.via(tweeny::easing::quarticInOut);
      break;
    case Tween::EasingType_::quinticIn:
      tween_ = tween_.via(tweeny::easing::quinticIn);
      break;
    case Tween::EasingType_::quinticOut:
      tween_ = tween_.via(tweeny::easing::quinticOut);
      break;
    case Tween::EasingType_::quinticInOut:
      tween_ = tween_.via(tweeny::easing::quinticInOut);
      break;
    // case Tween::EasingType_::sinuisodal:
    //   tween_ = tween_.via(tweeny::easing::sinuisodal);
    //   break;
    case Tween::EasingType_::exponentialIn:
      tween_ = tween_.via(tweeny::easing::exponentialIn);
      break;
    case Tween::EasingType_::exponentialOut:
      tween_ = tween_.via(tweeny::easing::exponentialOut);
      break;
    case Tween::EasingType_::exponentialInOut:
      tween_ = tween_.via(tweeny::easing::exponentialInOut);
      break;
    case Tween::EasingType_::circularIn:
      tween_ = tween_.via(tweeny::easing::circularIn);
      break;
    case Tween::EasingType_::circularOut:
      tween_ = tween_.via(tweeny::easing::circularOut);
      break;
    case Tween::EasingType_::circularInOut:
      tween_ = tween_.via(tweeny::easing::circularInOut);
      break;
    case Tween::EasingType_::backIn:
      tween_ = tween_.via(tweeny::easing::backIn);
      break;
    case Tween::EasingType_::backOut:
      tween_ = tween_.via(tweeny::easing::backOut);
      break;
    case Tween::EasingType_::backInOut:
      tween_ = tween_.via(tweeny::easing::backInOut);
      break;
    case Tween::EasingType_::elasticIn:
      tween_ = tween_.via(tweeny::easing::elasticIn);
      break;
    case Tween::EasingType_::elasticOut:
      tween_ = tween_.via(tweeny::easing::elasticOut);
      break;
    case Tween::EasingType_::elasticInOut:
      tween_ = tween_.via(tweeny::easing::elasticInOut);
      break;
    case Tween::EasingType_::bounceIn:
      tween_ = tween_.via(tweeny::easing::bounceIn);
      break;
    case Tween::EasingType_::bounceOut:
      tween_ = tween_.via(tweeny::easing::bounceOut);
      break;
    case Tween::EasingType_::bounceInOut:
      tween_ = tween_.via(tweeny::easing::bounceInOut);
      break;
    default:
      throw new Exception("Unknown easing type!");
  }

  // Make sure there are no unused keys
  reader.EnsureAccessedAllKeys();

  ROS_DEBUG_NAMED("Tween",
                  "Initialized with params body(%p %s) "
                  "start ({%f,%f,%f}) "
                  "end ({%f,%f,%f}) "
                  "duration %f "
                  "mode: %s [%d] "
                  "easing: %s\n",
                  body_, body_->name_.c_str(), start_.x, start_.y, start_.theta,
                  delta_.x, delta_.y, delta_.theta, duration_, mode.c_str(),
                  (int)mode_, easing.c_str());
}

void Tween::TriggerCallback(const std_msgs::Bool& msg) {
  triggered_ = msg.data;
}

void Tween::BeforePhysicsStep(const Timekeeper& timekeeper) {
  std::array<double, 3> v =
      tween_.step((uint32)(timekeeper.GetStepSize() * 1000.0));
  ROS_DEBUG_THROTTLE_NAMED(1.0, "Tween", "value %f,%f,%f step %f progress %f",
                           v[0], v[1], v[2], timekeeper.GetStepSize(),
                           tween_.progress());
  body_->physics_body_->SetTransform(b2Vec2(start_.x + v[0], start_.y + v[1]),
                                     start_.theta + v[2]);
  // Tell Box2D to update the AABB and check for collisions for this object
  body_->physics_body_->SetAwake(true);

  // Yoyo back and forth
  if (mode_ == Tween::ModeType_::YOYO) {
    if (tween_.progress() >= 1.0f) {
      tween_.backward();
    } else if (tween_.progress() <= 0.001f) {
      tween_.forward();
    }
  }

  // Teleport back in loop mode
  if (mode_ == Tween::ModeType_::LOOP) {
    if (tween_.progress() >= 1.0f) {
      tween_.seek(0);
    }
  }

  // Handle external trigger
  if (mode_ == Tween::ModeType_::TRIGGER) {
    if (triggered_) {
      tween_.forward();
    } else {
      tween_.backward();
    }
  }
}
}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Tween, flatland_server::ModelPlugin)