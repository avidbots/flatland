/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 layer.h
 * @brief	 Defines flatland Layer
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

#ifndef FLATLAND_SERVER_LAYER_H
#define FLATLAND_SERVER_LAYER_H

#include <Box2D/Box2D.h>
#include <flatland_server/entity.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <flatland_server/body.h>

namespace flatland_server {

class Layer : public Entity {
 public:
  uint8_t layer_index_;
  cv::Mat bitmap_;
  double resolution_;
  double occupied_thresh_;
  double free_thresh_;

  std::vector<b2EdgeShape> extracted_edges;  // edges extracted from bitmap
  Body *body_;

  /**
   * @brief Constructor for the Layer class. All data required for
   * initialization should be passed in here
   * @param[in] physics_world Pointer to the box2d physics world
   * @param[in] layer_index Unique index of the layer
   * @param[in] name Name of the layer
   * @param[in] bitmap Matrix containing the map image
   * @param[in] color Color in the form of r, g, b, a, used for visualization
   * @param[in] origin Coordinate of the lower left corner of the image, in the
   * form of x, y, theta
   * @param[in] resolution Resolution of the map image in meters per pixel
   * @param[in] occupied_thresh Threshold indicating obstacle if above
   * @param[in] free_thresh Threshold indicating no obstale if below
   */
  Layer(b2World *physics_world, uint8_t layer_index, const std::string &name,
        const cv::Mat &bitmap, const std::array<double, 4> &color,
        const std::array<double, 3> &origin, double resolution,
        double occupied_thresh, double free_thresh);

  /**
   * @brief Destructor for the layer class
   */
  ~Layer();

  /**
   * @brief Return the type of entity
   */
  virtual Type type() { return Type::LAYER; }

  /**
   * @brief Vectorize the bitmap into line segments usable by physics simulator
   */
  void vectorize_bitmap();

  /**
   * @brief Apply the necessary transformations and load the map into the
   * physics simulator
   */
  void load_edges();

  /**
   * @brief Factory method to instantiate a layer
   * @param[in] physics_world Pointer to the box2d physics world
   * @param[in] layer_index Index of the layer, in the order of yaml definition
   * @param[in] world_yaml_dir Path to the directory containing the world yaml
   * file, this is used to calculate the path to the layermap yaml file
   * @param[in] layer_node YAML node containing data for a layer
   */
  static Layer *make_layer(b2World *physics_world, uint8_t layer_index,
                           const boost::filesystem::path &world_yaml_dir,
                           const YAML::Node &layer_node);
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
