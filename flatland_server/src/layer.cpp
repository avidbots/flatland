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
 #include <string>
 #include <yaml-cpp/yaml.h>
 #include <opencv2/opencv.hpp>
 #include <flatland_server/layer.h>
 #include <flatland_server/exceptions.h>

namespace flatland_server {

void Layer::load_layer(const boost::filesystem::path &world_yaml_dir, 
  const YAML::Node &layer_node) {

  boost::filesystem::path map_yaml_path;

  if (layer_node["map"]) {
    map_yaml_path = 
      boost::filesystem::path(layer_node["map"].as<std::string>());
  }
  else {
    throw YAMLException("Invalid \"properties\" in " + name_ + " layer");
  }

  if (layer_node["color"] &&
    layer_node["color"].IsSequence()&&
    layer_node["color"].size() == 4) {

    color_[0] = layer_node["color"][0].as<double>();
    color_[1] = layer_node["color"][1].as<double>();
    color_[2] = layer_node["color"][2].as<double>();
    color_[3] = layer_node["color"][3].as<double>();
  }
  else {
    throw YAMLException("Invalid \"color\" in " + name_ + " layer");
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
    throw YAMLException("Error loading " + map_yaml_path.string(), 
      e.msg, e.mark);
  }

  if (yaml["image"]) {
    boost::filesystem::path image_path(yaml["image"].as<std::string>());

    if (image_path.string().front() != '/') {
      image_path = map_yaml_path.parent_path() / image_path;
    }

    cv::Mat map = cv::imread(image_path.string(), CV_LOAD_IMAGE_GRAYSCALE);
    map.convertTo(bitmap_, CV_32FC1, 1.0 / 255.0);
  }
  else{
    throw YAMLException("Invalid \"image\" in " + name_ + " layer");
  }

  if (yaml["resolution"]) {
    resolution_ = yaml["resolution"].as<double>();
  }
  else {
    throw YAMLException("Invalid \"resolution\" in " + name_ + " layer");
  }

  if (yaml["origin"] && yaml["origin"].IsSequence() &&
    yaml["origin"].size() == 3) {
    origin_[0] = yaml["origin"][0].as<double>();
    origin_[1] = yaml["origin"][1].as<double>();
    origin_[2] = yaml["origin"][2].as<double>();
  }
  else {
    throw YAMLException("Invalid \"origin\" in " + name_ + " layer");
  }

  if (yaml["occupied_thresh"]) {
    occupied_thresh_ = yaml["occupied_thresh"].as<double>();
  }
  else {
    throw YAMLException("Invalid \"occupied_thresh\" in " + 
      name_ +" layer");
  }

  if (yaml["free_thresh"]) {
    free_thresh_ = yaml["free_thresh"].as<double>();
  }
  else {
    throw YAMLException("Invalid \"free_thresh\" in " + name_ + " layer");
  }

  vectorize_bitmap();
}

void Layer::vectorize_bitmap() {
  cv::Mat padded_map, obstable_map;
  cv::copyMakeBorder(bitmap_, padded_map, 1, 1, 1, 1, cv::BORDER_CONSTANT, 0);

  // obstacle map is now binary
  cv::inRange(padded_map, occupied_thresh_, 1.0, obstable_map);

  // loop through all the rows, looking at 2 at once
  for (int i = 0; i < obstable_map.rows; i++) {
    cv::Mat row1 = obstable_map.row(i);
    cv::Mat row2 = obstable_map.row(i + 1);
    cv::Mat diff, non_zeros;

    // if the two row are the same value, there is no edge
    // if the two rows are not the same value, there is an edge
    // result is still binary, either 1 or 0
    cv::absdiff(row1, row2, diff);

    int start = 0, end = 0;

    // find all the walls, put the connected walls as a single line segment
    for (int j = 0; j < diff.total(); j++) {

      int edge_exists = diff.at<int>(0, j);

      if (edge_exists && start == end) {
        start = j;
      } 
      else if (!edge_exists || j == diff.total() - 1) {
        end = j;

        // this is your start and end
        b2FixtureDef fixture;
        b2EdgeShape edge;

        edge.Set(b2Vec2(start, i), b2Vec2(end, i)); // TODO
        fixture.shape = &edge;

        physics_body_->CreateFixture(&fixture);

        start = end;
      }
    }
  }

  // loop through all the columns, looking at 2 at once
  for (int i = 0; i < obstable_map.cols; i++) {
    cv::Mat col1 = obstable_map.col(i);
    cv::Mat col2 = obstable_map.col(i + 1);
    cv::Mat diff, non_zeros;

    cv::absdiff(col1, col2, diff);

    int start = 0, end = 0;

    for (int j = 0; j < diff.total(); j++) {

      int edge_exists = diff.at<int>(j, 0);

      if (edge_exists && start == end) {
        start = j;
      } 
      else if (!edge_exists || j == diff.total() - 1) {
        end = j;

        b2FixtureDef fixture;
        b2EdgeShape edge;

        edge.Set(b2Vec2(i, start), b2Vec2(i, end)); // TODO
        fixture.shape = &edge;

        physics_body_->CreateFixture(&fixture);

        start = end;
      }
    }
  }
}

};  // namespace flatland_server