/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name   spawn_model_tool.h
 * @brief  Rviz compatible tool for spawning flatland model
 * @author Joseph Duchesne
 * @author Mike Brousseau
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

#ifndef SPAWN_MODEL_TOOL_H
#define SPAWN_MODEL_TOOL_H

#include <rviz/tool.h>
#include <memory>
#include <vector>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OgreVector3.h>

#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include "rviz/ogre_helpers/arrow.h"

namespace flatland_viz {
/**
 * @name                SpawnModelTool
 * @brief               Every tool which can be added to the tool bar is a
 * subclass of rviz::Tool.
 */
class SpawnModelTool : public rviz::Tool {
  Q_OBJECT

 public:
  SpawnModelTool();
  ~SpawnModelTool();

  /**
   * @name                BeginPlacement
   * @brief               Begin the placement phase, model follows cursor, left
   * click deposits model and starts rotation phase
   */
  void BeginPlacement();
  /**
   * @name                SavePath
   * @brief               Called from dialog to save path
   */
  void SavePath(QString p);
  /**
   * @name                SaveName
   * @brief               Called from dialog to save name
   */
  void SaveName(QString n);
  /**
  * @name                SpawnModelInFlatland
  * @brief               Spawns a model using ros service
  */
  void SpawnModelInFlatland();

 private:
  /**
   * @name                onInitialize
   * @brief               Initializes tools currently loaded when rviz starts
   */
  virtual void onInitialize();
  /**
  virtual void activate();
   * @name                activate
   * @brief               Launch the model dialog
   */
  virtual void activate();
  /**
   * @name                deactivate
   * @brief               Cleanup when tool is removed
   */
  virtual void deactivate();
  /**
   * @name                processMouseEvent
   * @brief               Main loop of tool
   * @param event         Mouse event
   */
  virtual int processMouseEvent(rviz::ViewportMouseEvent &event);
  /**
  * @name                SetMovingModelColor
  * @brief               Set the color of the moving model
  * @param c             QColor to set the 3d model
  */
  void SetMovingModelColor(QColor c);
  /**
   * @name               LoadPreview
   * @brief              Load a vector preview of the model
   */
  void LoadPreview();
  /**
   * @name               LoadPolygonFootprint
   * @brief              Load a vector preview of the model's polygon footprint
   * @param footprint    The footprint yaml node
   * @param pose         x,y,theta pose of footprint
   */
  void LoadPolygonFootprint(flatland_server::YamlReader &footprint,
                            const flatland_server::Pose pose);
  /**
   * @name               LoadCircleFootprint
   * @brief              Load a vector preview of the model's circle footprint
   * @param footprint    The footprint yaml node
   * @param pose         x,y,theta pose of footprint
   */
  void LoadCircleFootprint(flatland_server::YamlReader &footprint,
                           const flatland_server::Pose pose);

  Ogre::Vector3
      intersection;     // location cursor intersects ground plane, ie the
                        // location to create the model
  float initial_angle;  // the angle to create the model at
  Ogre::SceneNode *moving_model_node_;  // the node for the 3D object
  enum ModelState { m_hidden, m_dragging, m_rotating };
  ModelState model_state;  // model state, first hidden, then dragging to
                           // intersection point, then rotating to desired angle
  static QString path_to_model_file_;  // full path to model file (yaml)
  static QString model_name;  // base file name with path and extension removed

 protected:
  rviz::Arrow *arrow_;        // Rviz 3d arrow to show axis of rotation
  ros::NodeHandle nh;         // ros service node handle
  ros::ServiceClient client;  // ros service client
  std::vector<std::shared_ptr<rviz::BillboardLine>> lines_list_;
};

}  // end namespace flatland_viz

#endif  // SPAWN_MODEL_TOOL_H
