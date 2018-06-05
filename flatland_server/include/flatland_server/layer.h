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
#include <flatland_server/body.h>
#include <flatland_server/collision_filter_registry.h>
#include <flatland_server/entity.h>
#include <flatland_server/types.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <string>

namespace flatland_server {

/**
 * This class defines a layer in the simulation world which simulates the
 * environment in the world
 */
class Layer : public Entity {
 public:
  std::vector<std::string> names_;  ///< list of layer names

  Body *body_ = nullptr;
  CollisionFilterRegistry *cfr_;  ///< collision filter registry
  std::string viz_name_;          ///< for visualization

  /**
   * @brief Constructor for the Layer class for initialization using a image
   * map file
   * @param[in] physics_world Pointer to the box2d physics world
   * @param[in] cfr Collision filter registry
   * @param[in] names A list of names for the layer, the first name is used
   * for the name of the body
   * @param[in] color Color in the form of r, g, b, a, used for visualization
   * @param[in] origin Coordinate of the lower left corner of the image, in the
   * form of x, y, theta
   * @param[in] occupied_thresh Threshold indicating obstacle if above
   * @param[in] bitmap Matrix containing the map image
   * @param[in] resolution Resolution of the map image in meters per pixel
   * @param[in] properties A YAML node containing properties for plugins to use
   */
  Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
        const std::vector<std::string> &names, const Color &color,
        const Pose &origin, const cv::Mat &bitmap, double occupied_thresh,
        double resolution, const YAML::Node &properties);

  /**
   * @brief Constructor for the Layer class for initialization using line
   * segments
   * @param[in] physics_world Pointer to the box2d physics world
   * @param[in] cfr Collision filter registry
   * @param[in] names A list of names for the layer, the first name is used
   * for the name of the body
   * @param[in] color Color in the form of r, g, b, a, used for visualization
   * @param[in] origin Coordinate of the lower left corner of the image, in the
   * form of x, y, theta
   * @param[in] line_segments List of line segments
   * @param[in] scale Scale to apply to the line segment end points, works in
   * the same way as resolution
   * @param[in] properties A YAML node containing properties for plugins to use
   */
  Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
        const std::vector<std::string> &names, const Color &color,
        const Pose &origin, const std::vector<LineSegment> &line_segments,
        double scale, const YAML::Node &properties);

  /**
  * @brief Constructor for the Layer class for initialization with no static
  * map in it
  * @param[in] physics_world Pointer to the box2d physics world
  * @param[in] cfr Collision filter registry
  * @param[in] names A list of names for the layer, the first name is used
  * for the name of the body
  * @param[in] properties A YAML node containing properties for plugins to use
  */
  Layer(b2World *physics_world, CollisionFilterRegistry *cfr,
        const std::vector<std::string> &names, const Color &color,
        const YAML::Node &properties);

  /**
   * @brief Destructor for the layer class
   */
  ~Layer();

  /**
   * @return The list of names the layer has
   */
  const std::vector<std::string> &GetNames() const;

  /**
   * @return The collision filter registrar
   */
  const CollisionFilterRegistry *GetCfr() const;

  Body *GetBody();

  /**
   * @brief Return the type of entity
   * @return type indicating it is a layer
   */
  EntityType Type() const { return EntityType::LAYER; }

  /**
   * @brief Load the map by extracting edges from images
   * @param[in] bitmap OpenCV Image
   * @param[in] occupied_thresh Threshold indicating obstacle if above
   * @param[in] resolution Resolution of the map image in meters per pixel
   */
  void LoadFromBitmap(const cv::Mat &bitmap, double occupied_thresh,
                      double resolution);

  /**
   * @brief Visualize layer for debugging purposes
   */
  void DebugVisualize() const override;

  /**
   * @brief log debug messages for the layer
   */
  void DebugOutput() const override;

  /**
   * @brief Read line segments from a file, each line of a file represents a
   * line segment in the form of x1 y1 x2 y2
   * @param[in] file_path Path to the file
   * @param[out] line_segments Line segments obtained from the file
   */
  static void ReadLineSegmentsFile(const std::string &file_path,
                                   std::vector<LineSegment> &line_segments);

  /**
   * @brief Factory method to instantiate a layer, throws exceptions upon
   * failure
   * @param[in] physics_world Pointer to the box2d physics world
   * @param[in] cfr Collision filter registry
   * @param[in] map_path Path to the file containing layer data
   * @param[in] names A list of names for the layer, the first name is used
   * for the name of the body. All names are registered in the CFR. Multiple
   * names allow the physics body to be used as if there are multiple layers
   * @param[in] color Color of the layer
   * file, this is used to calculate the path to the layermap yaml file
   * @param[in] properties A YAML node containing properties for plugins to use
   * @return A new layer
   */
  static Layer *MakeLayer(b2World *physics_world, CollisionFilterRegistry *cfr,
                          const std::string &map_path,
                          const std::vector<std::string> &names,
                          const Color &color, const YAML::Node &properties);
};
};      // namespace flatland_server
#endif  // FLATLAND_SERVER_WORLD_H
