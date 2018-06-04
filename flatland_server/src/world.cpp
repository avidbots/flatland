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
 * @brief	 Loads world file
 * @author Joseph Duchesne
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
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <flatland_server/yaml_reader.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <map>
#include <string>

namespace flatland_server {

World::World()
    : gravity_(0, 0),
      service_paused_(false),
      int_marker_manager_(&models_, &plugin_manager_) {
  physics_world_ = new b2World(gravity_);
  physics_world_->SetContactListener(this);
}

World::~World() {
  ROS_INFO_NAMED("World", "Destroying world...");

  // The order of things matters in the destructor. The contact listener is
  // removed first to avoid the triggering the contact functions in plugin
  // manager which might cause it to work with deleted layers/models.
  physics_world_->SetContactListener(nullptr);

  // the physics body of layers are set to null because there are tons of
  // fixtures in a layer and it is too slow for the destroyBody method to remove
  // them since the AABB tree gets restructured everytime a fixture is removed
  // The memory will later be freed by deleting the world
  for (auto &layer : layers_) {
    if (layer->body_ != nullptr) {
      layer->body_->physics_body_ = nullptr;
    }
    delete layer;
  }

  // The bodies of models are not set to null like layers because there aren't
  // nearly as many fixtures, and we might hide some memory problems by using
  // the shortcut
  for (unsigned int i = 0; i < models_.size(); i++) {
    delete models_[i];
  }

  // This frees the entire Box2D world with everything in it
  delete physics_world_;

  ROS_INFO_NAMED("World", "World destroyed");
}

void World::Update(Timekeeper &timekeeper) {
  if (!IsPaused()) {
    plugin_manager_.BeforePhysicsStep(timekeeper);
    physics_world_->Step(timekeeper.GetStepSize(), physics_velocity_iterations_,
                         physics_position_iterations_);
    timekeeper.StepTime();
    plugin_manager_.AfterPhysicsStep(timekeeper);
  }
  int_marker_manager_.update();
}

void World::BeginContact(b2Contact *contact) {
  plugin_manager_.BeginContact(contact);
}

void World::EndContact(b2Contact *contact) {
  plugin_manager_.EndContact(contact);
}

void World::PreSolve(b2Contact *contact, const b2Manifold *oldManifold) {
  plugin_manager_.PreSolve(contact, oldManifold);
}

void World::PostSolve(b2Contact *contact, const b2ContactImpulse *impulse) {
  plugin_manager_.PostSolve(contact, impulse);
}

World *World::MakeWorld(const std::string &yaml_path) {
  YamlReader world_reader = YamlReader(yaml_path);
  YamlReader prop_reader = world_reader.Subnode("properties", YamlReader::MAP);
  int v = prop_reader.Get<int>("velocity_iterations", 10);
  int p = prop_reader.Get<int>("position_iterations", 10);
  prop_reader.EnsureAccessedAllKeys();

  World *w = new World();

  w->world_yaml_dir_ = boost::filesystem::path(yaml_path).parent_path();
  w->physics_velocity_iterations_ = v;
  w->physics_position_iterations_ = p;

  try {
    YamlReader layers_reader = world_reader.Subnode("layers", YamlReader::LIST);
    YamlReader models_reader =
        world_reader.SubnodeOpt("models", YamlReader::LIST);
    YamlReader world_plugin_reader =
        world_reader.SubnodeOpt("plugins", YamlReader::LIST);
    world_reader.EnsureAccessedAllKeys();
    w->LoadLayers(layers_reader);
    w->LoadModels(models_reader);
    w->LoadWorldPlugins(world_plugin_reader, w, world_reader);
  } catch (const YAMLException &e) {
    ROS_FATAL_NAMED("World", "Error loading from YAML");
    delete w;
    throw e;
  } catch (const PluginException &e) {
    ROS_FATAL_NAMED("World", "Error loading plugins");
    delete w;
    throw e;
  } catch (const Exception &e) {
    ROS_FATAL_NAMED("World", "Error loading world");
    delete w;
    throw e;
  }
  return w;
}

void World::LoadLayers(YamlReader &layers_reader) {
  // loop through each layer and parse the data
  for (int i = 0; i < layers_reader.NodeSize(); i++) {
    YamlReader reader = layers_reader.Subnode(i, YamlReader::MAP);
    YamlReader name_reader = reader.Subnode("name", YamlReader::NO_CHECK);

    // allow names to be either a just a string or a list of strings
    std::vector<std::string> names;
    if (name_reader.Node().IsSequence()) {
      names = name_reader.AsList<std::string>(1, -1);
    } else {
      names.push_back(name_reader.As<std::string>());
    }

    if (cfr_.LayersCount() + names.size() > cfr_.MAX_LAYERS) {
      throw YAMLException(
          "Unable to add " + std::to_string(names.size()) +
          " additional layer(s) {" + boost::algorithm::join(names, ", ") +
          "}, current layers count is " + std::to_string(cfr_.LayersCount()) +
          ", max allowed is " + std::to_string(cfr_.MAX_LAYERS));
    }

    boost::filesystem::path map_path(reader.Get<std::string>("map", ""));
    Color color = reader.GetColor("color", Color(1, 1, 1, 1));
    auto properties =
        reader.SubnodeOpt("properties", YamlReader::NodeTypeCheck::MAP).Node();
    reader.EnsureAccessedAllKeys();

    for (const auto &name : names) {
      if (cfr_.RegisterLayer(name) == cfr_.LAYER_ALREADY_EXIST) {
        throw YAMLException("Layer with name " + Q(name) + " already exists");
      }
    }

    if (map_path.string().front() != '/' && map_path.string().length() > 0) {
      map_path = world_yaml_dir_ / map_path;
    }

    ROS_INFO_NAMED("World", "Loading layer \"%s\" from path=\"%s\"",
                   names[0].c_str(), map_path.string().c_str());

    Layer *layer = Layer::MakeLayer(physics_world_, &cfr_, map_path.string(),
                                    names, color, properties);
    layers_name_map_.insert(
        std::pair<std::vector<std::string>, Layer *>(names, layer));
    layers_.push_back(layer);

    ROS_INFO_NAMED("World", "Layer \"%s\" loaded", layer->name_.c_str());
    layer->DebugOutput();
  }
}

void World::LoadModels(YamlReader &models_reader) {
  if (!models_reader.IsNodeNull()) {
    for (int i = 0; i < models_reader.NodeSize(); i++) {
      YamlReader reader = models_reader.Subnode(i, YamlReader::MAP);

      std::string name = reader.Get<std::string>("name");
      std::string ns = reader.Get<std::string>("namespace", "");
      Pose pose = reader.GetPose("pose", Pose(0, 0, 0));
      std::string path = reader.Get<std::string>("model");
      reader.EnsureAccessedAllKeys();
      LoadModel(path, ns, name, pose);
    }
  }
}

void World::LoadWorldPlugins(YamlReader &world_plugin_reader, World *world,
                             YamlReader &world_config) {
  if (!world_plugin_reader.IsNodeNull()) {
    for (int i = 0; i < world_plugin_reader.NodeSize(); i++) {
      YamlReader reader = world_plugin_reader.Subnode(i, YamlReader::MAP);
      ROS_INFO_NAMED("World", "loading world_plugin");
      plugin_manager_.LoadWorldPlugin(world, reader, world_config);
    }
  }
}
void World::LoadModel(const std::string &model_yaml_path, const std::string &ns,
                      const std::string &name, const Pose &pose) {
  // ensure no duplicate model names
  if (std::count_if(models_.begin(), models_.end(),
                    [&](Model *m) { return m->name_ == name; }) >= 1) {
    throw YAMLException("Model with name " + Q(name) + " already exists");
  }

  boost::filesystem::path abs_path(model_yaml_path);
  if (model_yaml_path.front() != '/') {
    abs_path = world_yaml_dir_ / abs_path;
  }

  ROS_INFO_NAMED("World", "Loading model from path=\"%s\"",
                 abs_path.string().c_str());

  Model *m =
      Model::MakeModel(physics_world_, &cfr_, abs_path.string(), ns, name);
  m->TransformAll(pose);

  try {
    for (int i = 0; i < m->plugins_reader_.NodeSize(); i++) {
      YamlReader plugin_reader = m->plugins_reader_.Subnode(i, YamlReader::MAP);
      plugin_manager_.LoadModelPlugin(m, plugin_reader);
    }
  } catch (const YAMLException &e) {
    plugin_manager_.DeleteModelPlugin(m);
    delete m;
    throw e;
  } catch (const PluginException &e) {
    plugin_manager_.DeleteModelPlugin(m);
    delete m;
    throw e;
  }

  models_.push_back(m);

  visualization_msgs::MarkerArray body_markers;
  for (size_t i = 0; i < m->bodies_.size(); i++) {
    DebugVisualization::Get().BodyToMarkers(
        body_markers, m->bodies_[i]->physics_body_, 1.0, 0.0, 0.0, 1.0);
  }
  int_marker_manager_.createInteractiveMarker(name, pose, body_markers);

  ROS_INFO_NAMED("World", "Model \"%s\" loaded", m->name_.c_str());
  m->DebugOutput();
}

void World::DeleteModel(const std::string &name) {
  bool found = false;

  for (unsigned int i = 0; i < models_.size(); i++) {
    // name is unique, so there will only be one object with this name
    if (models_[i]->GetName() == name) {
      // delete the plugins associated with the model
      plugin_manager_.DeleteModelPlugin(models_[i]);
      delete models_[i];
      models_.erase(models_.begin() + i);
      int_marker_manager_.deleteInteractiveMarker(name);
      found = true;
      break;
    }
  }

  if (!found) {
    throw Exception("Flatland World: failed to delete model, model with name " +
                    Q(name) + " does not exist");
  }
}

void World::MoveModel(const std::string &name, const Pose &pose) {
  // Find desired model
  bool found = false;

  for (unsigned int i = 0; i < models_.size(); i++) {
    if (models_[i]->GetName() == name) {
      // move the model
      models_[i]->SetPose(pose);
      found = true;
      break;
    }
  }

  if (!found) {
    throw Exception("Flatland World: failed to move model, model with name " +
                    Q(name) + " does not exist");
  }
}

void World::Pause() { service_paused_ = true; }

void World::Resume() { service_paused_ = false; }

void World::TogglePaused() { service_paused_ = !service_paused_; }

bool World::IsPaused() {
  return service_paused_ || int_marker_manager_.isManipulating();
}

void World::DebugVisualize(bool update_layers) {
  if (update_layers) {
    for (const auto &layer : layers_) {
      layer->DebugVisualize();
    }
  }

  for (const auto &model : models_) {
    model->DebugVisualize();
  }
}
};  // namespace flatland_server
