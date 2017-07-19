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
#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/algorithm/string/join.hpp>
#include <cmath>

using namespace flatland_server;

namespace flatland_plugins {

void Laser::OnInitialize(const YAML::Node &config) {
  ParseParameters(config);
  scan_publisher = nh_.advertise<sensor_msgs::LaserScan>(topic_, 1);
  viz_markers_publisher =
      nh_.advertise<visualization_msgs::Marker>("scan_viz", 1, true);
  // tf_body_to_laser =
  //     b2Transform(b2Vec2(origin_[0], origin_[1]), b2Rot(origin_[2]));

  double c = cos(origin_[2]);
  double s = sin(origin_[2]);
  double x = origin_[0], y = origin_[1];
  m_body_to_laser_ << c, -s, x, /**/ s, c, y, /**/ 0, 0, 1;
  std::cout << "m_body_to_laser_: " << std::endl
            << m_body_to_laser_ << std::endl;

  zero_point_ = b2Vec2(0, 0);

  num_laser_points_ = std::lround((max_angle_ - min_angle_) / increment_) + 1;

  printf("A. %d\n", num_laser_points_);

  m_laser_points_ = Eigen::MatrixXf(3, num_laser_points_);
  m_world_laser_points_ = Eigen::MatrixXf(3, num_laser_points_);
  v_zero_point_ << 0, 0, 1;

  printf("B. %d\n", num_laser_points_);

  for (int i = 0; i < num_laser_points_; i++) {
    float angle = min_angle_ + i * increment_;

    float x = range_ * cos(angle);
    float y = range_ * sin(angle);

    m_laser_points_(0, i) = x;
    m_laser_points_(1, i) = y;
    m_laser_points_(2, i) = 1;
  }

  printf("C. %d\n", num_laser_points_);

  // std::cout << "m_laser_points_: " << std::endl << m_laser_points_ <<
  // std::endl;
  std::cout << "v_zero_point_: " << std::endl << v_zero_point_ << std::endl;

  laser_scan_.angle_min = min_angle_;
  laser_scan_.angle_max = max_angle_;
  laser_scan_.angle_increment = increment_;
  laser_scan_.time_increment = 0;
  laser_scan_.scan_time = 0;
  laser_scan_.range_min = 0;
  laser_scan_.range_max = range_;
  laser_scan_.ranges.resize(num_laser_points_);
  laser_scan_.intensities.resize(0);
  laser_scan_.header.seq = 0;
  laser_scan_.header.frame_id = frame_;

  printf("D. %d\n", num_laser_points_);

  // Broadcast transform between the body and laser
  geometry_msgs::TransformStamped static_tf;
  tf2::Quaternion q;
  q.setRPY(0, 0, origin_[2]);
  static_tf.header.stamp = ros::Time::now();
  static_tf.header.frame_id = body_->name_;
  static_tf.child_frame_id = frame_;
  static_tf.transform.translation.x = origin_[0];
  static_tf.transform.translation.y = origin_[1];
  static_tf.transform.translation.z = 0;
  static_tf.transform.rotation.x = q.x();
  static_tf.transform.rotation.y = q.y();
  static_tf.transform.rotation.z = q.z();
  static_tf.transform.rotation.w = q.w();
  tf_broadcaster.sendTransform(static_tf);

  printf("E. %d\n", num_laser_points_);

  ROS_INFO_NAMED("LaserPlugin", "Laser %s initialized", name_.c_str());
  body_->physics_body_->SetLinearVelocity(b2Vec2(3, 0));
}

void Laser::BeforePhysicsStep(const TimeKeeper &time_keeper) {
  body_->physics_body_->SetAngularVelocity(2);
  model_->DebugVisualize();

  // printf("1. %d\n", num_laser_points_);

  const b2Transform &t = body_->physics_body_->GetTransform();
  m_world_to_body_ << t.q.c, -t.q.s, t.p.x, t.q.s, t.q.c, t.p.y, 0, 0, 1;
  m_world_to_laser_ = m_world_to_body_ * m_body_to_laser_;

  // printf("2. %d\n", num_laser_points_);

  m_world_laser_points_ = m_world_to_laser_ * m_laser_points_;
  v_world_laser_origin_ = m_world_to_laser_ * v_zero_point_;


// printf("3. %d\n", num_laser_points_);
//   std::cout << "m_world_to_body_: " << std::endl
//             << m_world_to_body_ << std::endl;
//   std::cout << "m_world_to_laser_: " << std::endl
//             << m_world_to_laser_ << std::endl;
//   std::cout << "v_world_laser_origin_: " << std::endl
//             << v_world_laser_origin_ << std::endl;

  b2Vec2 laser_point;
  b2Vec2 laser_origin_point(v_world_laser_origin_(0), v_world_laser_origin_(1));

  markers_.points.clear();

  // printf("4. %d\n", num_laser_points_);

  for (int i = 0; i < num_laser_points_; ++i) {
    // printf("%d of %d, %d %d\n", i, num_laser_points_, m_world_laser_points_.rows(), m_world_laser_points_.cols());
    laser_point.x = m_world_laser_points_(0, i);
    laser_point.y = m_world_laser_points_(1, i);

    did_hit_ = false;
    point_hit_ = b2Vec2(0, 0);  // DEBUG

    model_->physics_world_->RayCast(this, laser_origin_point, laser_point);

    if (!did_hit_) {
      laser_scan_.ranges[i] = 9.99;
      // laser_scan_.ranges[i] = 2.99;
    } else {
      laser_scan_.ranges[i] = fraction_ * range_;
    }

    geometry_msgs::Point pt;
    pt.x = point_hit_.x;
    pt.y = point_hit_.y;
    pt.z = 0;
    markers_.points.push_back(pt);
  }

  laser_scan_.header.stamp = ros::Time::now();
  scan_publisher.publish(laser_scan_);

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
  if (!(fixture->GetFilterData().categoryBits & layers_bits_)) {
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
    frame_ = name_;
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
      origin_[1], origin_[2], range_, min_angle_, max_angle_, increment_,
      layers_bits_, boost::algorithm::join(layers, ",").c_str());
}
};

PLUGINLIB_EXPORT_CLASS(flatland_plugins::Laser, flatland_server::ModelPlugin)