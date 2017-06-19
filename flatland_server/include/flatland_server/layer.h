/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 world.h
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
#include <string>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

namespace flatland_server {
class Layer {
 public:
    std::string name_;
    std::array<double, 4> color_; // r, g, b, a
    std::array<double, 3> origin_;
    cv::Mat bitmap_;
    double resolution_;
    double occupied_thresh_;
    double free_thresh_;

    b2Body *physics_body_;

    Layer(b2World *physics_world, const std::string &name, const cv::Mat &bitmap, 
      const std::array<double, 4> &color, const std::array<double, 3> &origin,
      double &resolution, double &occupied_thresh, double &free_thresh);
    ~Layer();

  /* This class should be non-copyable. This will cause the destructor to be
      called twice for a given b2Body*/
    Layer(const Layer&) = delete;
    Layer& operator=(const Layer&) = delete;

    void vectorize_bitmap();

    static void parse_yaml_node(
      const boost::filesystem::path &world_yaml_dir, 
      const YAML::Node &layer_node,  
      cv::Mat *bitmap,
      std::array<double, 4> *color, std::array<double, 3> *origin,
      double *resolution, double *occupied_thresh, double *free_thresh);

    static Layer *make_layer(b2World *physics_world, 
      boost::filesystem::path world_yaml_dir, YAML::Node layer_node);
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
