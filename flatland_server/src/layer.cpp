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
#include <flatland_server/debug_visualization.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/geometry.h>
#include <flatland_server/layer.h>
#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#if CV_MAJOR_VERSION < 3
#define GREYSCALE CV_LOAD_IMAGE_GRAYSCALE
#else
#include <opencv2/imgcodecs.hpp>
#define GREYSCALE cv::ImreadModes::IMREAD_GRAYSCALE
#endif
#include <sstream>

namespace flatland_server {

Layer::Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
             const std::vector<std::string> &names, const Color &color,
             const Pose &origin, const cv::Mat &bitmap, double occupied_thresh,
             double resolution, const YAML::Node &properties)
    : Entity(physics_world, names[0]),
      names_(names),
      cfr_(cfr),
      viz_name_("layer/" + names[0]) {
  body_ = new Body(physics_world_, this, name_, color, origin, b2_staticBody,
                   properties);

  LoadFromBitmap(bitmap, occupied_thresh, resolution);
}

Layer::Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
             const std::vector<std::string> &names, const Color &color,
             const Pose &origin, const std::vector<LineSegment> &line_segments,
             double scale, const YAML::Node &properties)
    : Entity(physics_world, names[0]),
      names_(names),
      cfr_(cfr),
      viz_name_("layer/" + names[0]) {
  body_ = new Body(physics_world_, this, name_, color, origin, b2_staticBody,
                   properties);

  uint16_t category_bits = cfr_->GetCategoryBits(names_);

  for (const auto &line_segment : line_segments) {
    b2EdgeShape edge;
    edge.Set(line_segment.start.Box2D(), line_segment.end.Box2D());
    edge.m_vertex1 *= scale;
    edge.m_vertex2 *= scale;

    b2FixtureDef fixture_def;
    fixture_def.shape = &edge;
    fixture_def.filter.categoryBits = category_bits;
    fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
    // todo: add material information
    body_->physics_body_->CreateFixture(&fixture_def);
  }
}

Layer::Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
             const std::vector<std::string> &names, const Color &color,
             const YAML::Node &properties)
    : Entity(physics_world, names[0]),
      names_(names),
      cfr_(cfr),
      viz_name_("layer/" + names[0]) {}

Layer::~Layer() { delete body_; }

const std::vector<std::string> &Layer::GetNames() const { return names_; }

const CollisionFilterRegistry *Layer::GetCfr() const { return cfr_; }
Body *Layer::GetBody() { return body_; }

Layer *Layer::MakeLayer(b2World *physics_world, CollisionFilterRegistry *cfr,
                        const std::string &map_path,
                        const std::vector<std::string> &names,
                        const Color &color, const YAML::Node &properties) {
  if (map_path.length() > 0) {  // If there is a map in this layer
    YamlReader reader(map_path);
    reader.SetErrorInfo("layer " + Q(names[0]));

    std::string type = reader.Get<std::string>("type", "");

    if (type == "line_segments") {
      double scale = reader.Get<double>("scale");
      Pose origin = reader.GetPose("origin");
      boost::filesystem::path data_path(reader.Get<std::string>("data"));
      if (data_path.string().front() != '/') {
        data_path = boost::filesystem::path(map_path).parent_path() / data_path;
      }

      ROS_INFO_NAMED("Layer",
                     "layer \"%s\" loading line segments from path=\"%s\"",
                     names[0].c_str(), data_path.string().c_str());

      std::vector<LineSegment> line_segments;

      ReadLineSegmentsFile(data_path.string(), line_segments);

      return new Layer(physics_world, cfr, names, color, origin, line_segments,
                       scale, properties);

    } else {
      double resolution = reader.Get<double>("resolution");
      double occupied_thresh = reader.Get<double>("occupied_thresh");
      Pose origin = reader.GetPose("origin");

      boost::filesystem::path image_path(reader.Get<std::string>("image"));
      if (image_path.string().front() != '/') {
        image_path =
            boost::filesystem::path(map_path).parent_path() / image_path;
      }

      ROS_INFO_NAMED("Layer", "layer \"%s\" loading image from path=\"%s\"",
                     names[0].c_str(), image_path.string().c_str());

      cv::Mat map = cv::imread(image_path.string(), GREYSCALE);
      if (map.empty()) {
        throw YAMLException("Failed to load " + Q(image_path.string()) +
                            " in layer " + Q(names[0]));
      }

      cv::Mat bitmap;
      map.convertTo(bitmap, CV_32FC1, 1.0 / 255.0);

      return new Layer(physics_world, cfr, names, color, origin, bitmap,
                       occupied_thresh, resolution, properties);
    }
  } else {  // If the layer has no static obstacles
    return new Layer(physics_world, cfr, names, color, properties);
  }
}

void Layer::ReadLineSegmentsFile(const std::string &file_path,
                                 std::vector<LineSegment> &line_segments) {
  std::ifstream in_file(file_path);
  std::string line;
  int line_count = 0;

  line_segments.clear();

  if (in_file.fail()) {
    throw Exception("Flatland File: Failed to load " + Q(file_path));
  }

  while (std::getline(in_file, line)) {
    line_count++;
    std::stringstream ss(line);
    float n[4];

    for (unsigned int i = 0; i < 4; i++) {
      ss >> n[i];

      if (ss.fail()) {
        throw Exception(
            "Flatland File: Failed to read line segment from line " +
            std::to_string(line_count) + ", in file " +
            Q(boost::filesystem::path(file_path).filename().string()));
      }
    }

    line_segments.push_back(LineSegment(Vec2(n[0], n[1]), Vec2(n[2], n[3])));
  }
}

void Layer::LoadFromBitmap(const cv::Mat &bitmap, double occupied_thresh,
                           double resolution) {
  uint16_t category_bits = cfr_->GetCategoryBits(names_);

  auto add_edge = [&](double x1, double y1, double x2, double y2) {
    b2EdgeShape edge;
    double rows = bitmap.rows;
    double res = resolution;

    edge.Set(b2Vec2(res * x1, res * (rows - y1)),
             b2Vec2(res * x2, res * (rows - y2)));

    b2FixtureDef fixture_def;
    fixture_def.shape = &edge;
    fixture_def.filter.categoryBits = category_bits;
    fixture_def.filter.maskBits = fixture_def.filter.categoryBits;
    body_->physics_body_->CreateFixture(&fixture_def);
  };

  cv::Mat padded_map, obstacle_map;

  // thresholds the map, values between the occupied threshold and 1.0 are
  // considered to be occupied
  cv::inRange(bitmap, occupied_thresh, 1.0, obstacle_map);

  // pad the top and bottom of the map each with an empty row (255=white). This
  // helps to look at the transition from one row of pixel to another
  cv::copyMakeBorder(obstacle_map, padded_map, 1, 1, 0, 0, cv::BORDER_CONSTANT,
                     255);

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
    for (unsigned int j = 0; j <= diff.total(); j++) {
      bool edge_exists = false;
      if (j < diff.total()) {
        edge_exists = diff.at<uint8_t>(0, j);  // 255 maps to true
      }

      if (edge_exists && !started) {
        start = j;
        started = true;
      } else if (started && !edge_exists) {
        add_edge(start, i, j, i);

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

    for (unsigned int j = 0; j <= diff.total(); j++) {
      bool edge_exists = false;
      if (j < diff.total()) {
        edge_exists = diff.at<uint8_t>(j, 0);
      }

      if (edge_exists && !started) {
        start = j;
        started = true;
      } else if (started && !edge_exists) {
        add_edge(i, start, i, j);

        started = false;
      }
    }
  }
}

void Layer::DebugVisualize() const {
  // Don't try to visualized uninitalized layers
  if (viz_name_.length() == 0) {
    return;
  }

  DebugVisualization::Get().Reset(viz_name_);
  DebugVisualization::Get().Reset(viz_name_ + "_3d");

  if (body_ != nullptr) {
    DebugVisualization::Get().Visualize(viz_name_, body_->physics_body_,
                                        body_->color_.r, body_->color_.g,
                                        body_->color_.b, body_->color_.a);
    DebugVisualization::Get().VisualizeLayer(viz_name_ + "_3d", body_);
  }
}

void Layer::DebugOutput() const {
  std::string names = "{" + boost::algorithm::join(names_, ",") + "}";
  uint16_t category_bits = cfr_->GetCategoryBits(names_);

  ROS_DEBUG_NAMED("Layer",
                  "Layer %p: physics_world(%p) name(%s) names(%s) "
                  "category_bits(0x%X)",
                  this, physics_world_, name_.c_str(), names.c_str(),
                  category_bits);

  if (body_ != nullptr) {
    body_->DebugOutput();
  }
}

};  // namespace flatland_server
