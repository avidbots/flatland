/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	  laser.cpp
 * @brief   Laser plugin
 * @author  Chunshang Li
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

#include <flatland_plugins/laser.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/model_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <boost/algorithm/string/join.hpp>
#include <cmath>

using namespace flatland_server;

namespace flatland_plugins {

void Laser::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  scan_publisher = nh_.advertise<sensor_msgs::LaserScan>(topic_, 1);
  viz_markers_publisher = nh_.advertise<visualization_msgs::Marker>("scan_viz", 1, true);
  tf_body_to_laser =
      b2Transform(b2Vec2(origin_[0], origin_[1]), b2Rot(origin_[2]));

  zero_point_ = b2Vec2(0, 0);

  int num_points = std::lround((max_angle_ - min_angle_) / increment_) + 1;

  for (int i = 0; i < num_points; i++) {
    float angle = min_angle_ + i * increment_;

    float x = range_ * cos(angle);
    float y = range_ * sin(angle);
    printf("%f: %f, %f\n", angle, x, y);
    laser_points.push_back(b2Vec2(x, y));
  }

  laser_scan.angle_min = min_angle_;
  laser_scan.angle_max = max_angle_;
  laser_scan.angle_increment = increment_;
  laser_scan.time_increment = 0;
  laser_scan.scan_time = 0;
  laser_scan.range_min = 0;
  laser_scan.range_max = range_;
  laser_scan.ranges.resize(num_points);
  laser_scan.intensities.resize(0);
  laser_scan.header.seq = 0;
  laser_scan.header.frame_id = frame_;

  ROS_INFO_NAMED("LaserPlugin", "Laser %s initialized", name_.c_str());
  body_->physics_body_->SetLinearVelocity(b2Vec2(3, 0));
}

void Laser::BeforePhysicsStep(double timestep) {
  body_->physics_body_->SetAngularVelocity(2);
  model_->DebugVisualize();
  const b2Transform &tf_world_to_body = body_->physics_body_->GetTransform();

  markers_.points.clear();

  for (int i = 0; i < laser_points.size(); i++) {
    const b2Vec2 &point = laser_points[i];
    const b2Transform &tf_world_to_laser =
        b2MulT(tf_world_to_body, tf_body_to_laser);
    const b2Vec2 &laser_point_world = b2MulT(tf_world_to_laser, point);
    const b2Vec2 &laser_origin_world = b2MulT(tf_world_to_laser, zero_point_);

    model_->physics_world_->RayCast(this, laser_origin_world,
                                    laser_point_world);

    if (!did_hit_) {
      // SET NAN/INF?
      laser_scan.ranges[i] = NAN;
    } else {
      laser_scan.ranges[i] = fraction_ * range_;
    }

    geometry_msgs::Point pt;
    pt.x = point_hit_.x;
    pt.y = point_hit_.y;
    pt.z = 0;
    markers_.points.push_back(pt);
  }

  scan_publisher.publish(laser_scan);


  markers_.header.frame_id = "map";
  markers_.ns = "laser_markers";
  markers_.id = 100;
  markers_.action = 0;
  markers_.color.r = 1;
  markers_.color.g = 1;
  markers_.color.b = 1;
  markers_.color.a = 1;  
  markers_.scale.x = 0.1;
  markers_.scale.y = markers_.scale.x;
  markers_.scale.z = markers_.scale.x;
  markers_.type = 7;
  viz_markers_publisher.publish(markers_);
}

float Laser::ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                           const b2Vec2 &normal, float fraction) {
  if (fixture->GetFilterData().categoryBits != layers_bits_) {
    return -1.0f;
  }

  did_hit_ = true;
  point_hit_ = point;
  fraction_ = fraction;

  return fraction;
}

void Laser::DebugVisualize() {
  // output lines to debug visualize?

}

void Laser::ParseParameters(const YAML::Node &config) {
  const YAML::Node &n = config;
  std::string body_name;

  if (n["topic"]) {
    topic_ = n["topic"].as<std::string>();
  } else {
    topic_ = "/scan";
  }

  if (n["body"]) {
    body_name = n["body"].as<std::string>();
  } else {
    throw YAMLException("Missing \"body\" param");
  }

  if (n["frame"]) {
    frame_ = n["frame"].as<std::string>();
  } else {
    frame_ = "laser";
  }

  if (n["origin"] && n["origin"].IsSequence() && n["origin"].size() == 3) {
    origin_[0] = n["origin"][0].as<double>();
    origin_[1] = n["origin"][1].as<double>();
    origin_[2] = n["origin"][2].as<double>();
  } else if (n["origin"]) {
    throw YAMLException("Missing/invalid \"origin\" param");
  } else {
    origin_ = {0, 0, 0};
  }

  if (n["range"]) {
    range_ = n["range"].as<double>();
  } else {
    throw YAMLException("Missing \"range\" input");
  }

  if (n["angle"] && n["angle"].IsMap() && n["angle"]["min"] &&
      n["angle"]["max"] && n["angle"]["increment"]) {
    const YAML::Node &angle = n["angle"];
    min_angle_ = angle["min"].as<double>();
    max_angle_ = angle["max"].as<double>();
    increment_ = angle["increment"].as<double>();
  } else {
    throw YAMLException(
        "Missing/invalid \"angle\" param, must be a map with keys \"min\", "
        "\"max\", and \"increment\"");
  }

  if (max_angle_ > min_angle_ && max_angle_ > 0 && max_angle_ < 2 * M_PI &&
      min_angle_ > 0 && min_angle_ < 2 * M_PI) {
    throw YAMLException(
        "Invalid \"angle\" params, must have max > min, 0 < min < 2*PI, 0 < "
        "max < 2*PI");
  }

  std::vector<std::string> layers;
  if (n["layers"] && n["layers"].IsSequence()) {
    for (int i = 0; i < n["layers"].size(); i++) {
      layers.push_back(n["layers"][i].as<std::string>());
    }
  } else if (n["layers"]) {
    throw YAMLException("Invalid layers, must be a sequence");
  } else {
    layers = {"all"};
  }

  if (layers.size() == 1 && layers[0] == "all") {
    layers.clear();
    model_->cfr_->ListAllLayers(layers);
  }

  body_ = model_->GetBody(body_name);

  if (!body_) {
    throw YAMLException("Cannot find body with name " + body_name);
  }

  std::vector<std::string> failed_layers;
  layers_bits_ = model_->cfr_->GetCategoryBits(layers, &failed_layers);

  if (!failed_layers.empty()) {
    throw YAMLException("Cannot find layer(s): {" +
                        boost::algorithm::join(failed_layers, ",") + "}");
  }

  ROS_INFO_NAMED(
      "LaserPlugin",
      "Laser %s params: topic(%s) body(%s %p) origin(%f,%f,%f) range(%f) "
      "angle_min(%f) angle_max(%f) angle_increment(%f) layers(0x%u {%s})",
      name_.c_str(), topic_.c_str(), body_name.c_str(), body_, origin_[0],
      origin_[2], origin_[2], range_, min_angle_, max_angle_, increment_,
      layers_bits_, boost::algorithm::join(layers, ",").c_str());
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Laser, flatland_server::ModelPlugin)