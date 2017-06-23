/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.cpp
 * @brief	 implements flatland layer
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
#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/layer.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <flatland_server/debug_visualization.h>

namespace flatland_server {

Layer::Layer(b2World *physics_world, const std::string &name,
             const cv::Mat &bitmap, const std::array<double, 4> &color,
             const std::array<double, 3> &origin, const double &resolution,
             const double &occupied_thresh, const double &free_thresh)
    : name_(name),
      color_(color),
      origin_(origin),
      resolution_(resolution),
      occupied_thresh_(occupied_thresh),
      free_thresh_(free_thresh) {
  bitmap.copyTo(bitmap_);

  b2BodyDef body_def;
  body_def.type = b2_staticBody;

  physics_body_ = physics_world->CreateBody(&body_def);

  vectorize_bitmap();
  load_edges();

  DebugVisualization::get().visualize(std::string("layer_")+name, 
    physics_body_ , color_[0], color_[1], color_[2], color_[3]);

  ROS_INFO_NAMED("Layer", "Layer %s added", name_.c_str());
}

Layer::~Layer() { physics_body_->GetWorld()->DestroyBody(physics_body_); }

Layer *Layer::make_layer(b2World *physics_world,
                         boost::filesystem::path world_yaml_dir,
                         YAML::Node layer_node) {
  std::string name;
  cv::Mat bitmap;
  std::array<double, 4> color;
  std::array<double, 3> origin;
  double resolution, occupied_thresh, free_thresh;

  boost::filesystem::path map_yaml_path;

  if (layer_node["name"]) {
    name = layer_node["name"].as<std::string>();
  } else {
    throw YAMLException("Invalid \"properties\" in " + name + " layer");
  }

  if (layer_node["map"]) {
    map_yaml_path =
        boost::filesystem::path(layer_node["map"].as<std::string>());
  } else {
    throw YAMLException("Invalid \"properties\" in " + name + " layer");
  }

  if (layer_node["color"] && layer_node["color"].IsSequence() &&
      layer_node["color"].size() == 4) {
    color[0] = layer_node["color"][0].as<double>();
    color[1] = layer_node["color"][1].as<double>();
    color[2] = layer_node["color"][2].as<double>();
    color[3] = layer_node["color"][3].as<double>();
  } else {
    throw YAMLException("Invalid \"color\" in " + name + " layer");
  }

  // use absolute path if start with '/', use relative otherwise
  if (map_yaml_path.string().front() != '/') {
    map_yaml_path = world_yaml_dir / map_yaml_path;
  }

  // start parsing the map yaml file
  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(map_yaml_path.string());
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading " + map_yaml_path.string(), e.msg,
                        e.mark);
  }

  if (yaml["resolution"]) {
    resolution = yaml["resolution"].as<double>();
  } else {
    throw YAMLException("Invalid \"resolution\" in " + name + " layer");
  }

  if (yaml["origin"] && yaml["origin"].IsSequence() &&
      yaml["origin"].size() == 3) {
    origin[0] = yaml["origin"][0].as<double>();
    origin[1] = yaml["origin"][1].as<double>();
    origin[2] = yaml["origin"][2].as<double>();
  } else {
    throw YAMLException("Invalid \"origin\" in " + name + " layer");
  }

  if (yaml["occupied_thresh"]) {
    occupied_thresh = yaml["occupied_thresh"].as<double>();
  } else {
    throw YAMLException("Invalid \"occupied_thresh\" in " + name + " layer");
  }

  if (yaml["free_thresh"]) {
    free_thresh = yaml["free_thresh"].as<double>();
  } else {
    throw YAMLException("Invalid \"free_thresh\" in " + name + " layer");
  }

  if (yaml["image"]) {
    boost::filesystem::path image_path(yaml["image"].as<std::string>());

    if (image_path.string().front() != '/') {
      image_path = map_yaml_path.parent_path() / image_path;
    }

    cv::Mat map = cv::imread(image_path.string(), CV_LOAD_IMAGE_GRAYSCALE);
    if (map.empty()) {
      throw YAMLException("Failed to load " + image_path.string());
    }

    map.convertTo(bitmap, CV_32FC1, 1.0 / 255.0);
  } else {
    throw YAMLException("Invalid \"image\" in " + name + " layer");
  }

  return new Layer(physics_world, name, bitmap, color, origin, resolution,
                   occupied_thresh, free_thresh);
}

void Layer::vectorize_bitmap() {
  cv::Mat padded_map, obstacle_map;

  // thresholds the map, values between the occupied threshold and 1.0 are
  // considered to be occupied
  cv::inRange(bitmap_, occupied_thresh_, 1.0, obstacle_map);

  // cv::namedWindow( "Display window" );
  // cv::imshow( "Display window", bitmap_ );
  // cv::waitKey(0);

  // pad the top and bottom of the map each with an empty row (255=white). This 
  // helps to look at the transition from one row of pixel to another
  cv::copyMakeBorder(obstacle_map, padded_map, 1, 1, 0, 0, 
    cv::BORDER_CONSTANT, 255);

  // loop through all the rows, looking at 2 at once
  for (int i = 0; i < padded_map.rows - 1; i++) {
    cv::Mat row1 = padded_map.row(i);
    cv::Mat row2 = padded_map.row(i + 1);
    cv::Mat diff;

    // if the two row are the same value, there is no edge
    // if the two rows are not the same value, there is an edge
    // result is still binary, either 255 or 0
    cv::absdiff(row1, row2, diff);

    int start = 0;
    bool started = false;

    // find all the walls, put the connected walls as a single line segment
    for (int j = 0; j <= diff.total(); j++) {
      bool edge_exists = false;
      if (j < diff.total()) {
        edge_exists = diff.at<uint8_t>(0, j);  // 255 maps to true
      }

      if (edge_exists && !started) {
        start = j;
        started = true;
      } else if (started && !edge_exists) {
        b2EdgeShape edge123;
        edge123.Set(b2Vec2(start, i), b2Vec2(j, i));
        extracted_edges.push_back(edge123);

        started = false;
      }
    }
  }

  // pad the left and right of the map each with an empty column (255).
  cv::copyMakeBorder(obstacle_map, padded_map, 0, 0, 1, 1, cv::BORDER_CONSTANT,
                     255);

  // loop through all the columns, looking at 2 at once
  for (int i = 0; i < padded_map.cols - 1; i++) {
    cv::Mat col1 = padded_map.col(i);
    cv::Mat col2 = padded_map.col(i + 1);
    cv::Mat diff;

    cv::absdiff(col1, col2, diff);

    int start = 0;
    bool started = false;

    for (int j = 0; j <= diff.total(); j++) {
      bool edge_exists = false;
      if (j < diff.total()) {
        edge_exists = diff.at<uint8_t>(j, 0);
      }

      if (edge_exists && !started) {
        start = j;
        started = true;
      } else if (started && !edge_exists) {
        b2EdgeShape edge123;
        edge123.Set(b2Vec2(i, start), b2Vec2(i, j));
        extracted_edges.push_back(edge123);

        started = false;
      }
    }
  }
}

void Layer::load_edges() {
  // rotation from the map yaml origin is ignored
  RotateTranslate transform =
      Geometry::createTransform(origin_[0], origin_[1], 0);

  for (const auto &edge : extracted_edges) {
    b2EdgeShape edge_tf;

    // creates a copy
    b2Vec2 v1 = edge.m_vertex1;
    b2Vec2 v2 = edge.m_vertex2;

    v1.y = bitmap_.rows - v1.y;
    v2.y = bitmap_.rows - v2.y;

    v1.x = v1.x * resolution_;
    v1.y = v1.y * resolution_;
    v2.x = v2.x * resolution_;
    v2.y = v2.y * resolution_;

    b2Vec2 v1_tf = Geometry::transform(v1, transform);
    b2Vec2 v2_tf = Geometry::transform(v2, transform);

    edge_tf.Set(v1_tf, v2_tf);

    b2FixtureDef fixture_def;
    fixture_def.shape = &edge_tf;
    physics_body_->CreateFixture(&fixture_def);
  }
}

};  // namespace flatland_server
