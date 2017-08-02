/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2017 Avidbots Corp.
 * @name	 yaml_reader.h
 * @brief	 Defines yaml_reader
 * @author   Chunshang Li
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

#include <flatland_server/yaml_reader.h>

namespace flatland_server {

/**
*@brief Helper function to format the in message adding spaces and brackets
*/
void YamlReader::SetErrorLocationMsg(const std::string &msg) {
  if (msg.size() == 0) {
    in_ = "";
  }
  std::string msg_cpy = msg;
  boost::algorithm::to_lower(msg_cpy);
  in_ = " (in " + msg_cpy + ")";
}

std::string YamlReader::Q(const std::string &msg) { return "\"" + msg + "\""; }

YamlReader::YamlReader() : node_(YAML::Node()) { SetErrorLocationMsg(""); }

YamlReader::YamlReader(const YAML::Node &node, std::string error_location_msg)
    : node_(node) {
  SetErrorLocationMsg(error_location_msg);
}

YamlReader::YamlReader(const std::string &path,
                       std::string error_location_msg) {
  try {
    node_ = YAML::LoadFile(path);
  } catch (const YAML::BadFile &e) {
    throw YAMLException("File does not exist, path=" + Q(path), e);

  } catch (const YAML::ParserException &e) {
    throw YAMLException("Malformatted file, path=" + Q(path), e);

  } catch (const YAML::Exception &e) {
    throw YAMLException("Error loading file, path=" + Q(path), e);
  }

  SetErrorLocationMsg(error_location_msg);
}

YAML::Node YamlReader::Node() { return node_; }

bool YamlReader::IsNodeNull() { return node_.IsNull(); }

int YamlReader::NodeSize() { return node_.size(); }

YamlReader YamlReader::SubNode(int index, NodeTypeCheck type_check,
                               std::string error_location_msg) {
  if (!node_[index]) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  does not exist" + in_);
  }

  if (type_check == NodeTypeCheck::MAP && !node_[index].IsMap()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a map" + in_);

  } else if (type_check == NodeTypeCheck::LIST && !node_[index].IsSequence()) {
    throw YAMLException("Entry index=" + std::to_string(index) +
                        "  must be a list" + in_);
  }

  return YamlReader(node_[index], error_location_msg);
}

YamlReader YamlReader::SubNode(const std::string &key, NodeTypeCheck type_check,
                               std::string error_location_msg) {
  if (!node_[key]) {
    throw YAMLException("Entry key=" + Q(key) + " does not exist" + in_);
  }

  if (type_check == NodeTypeCheck::MAP && !node_[key].IsMap()) {
    throw YAMLException("Entry key=" + Q(key) + " must be a map" + in_);

  } else if (type_check == NodeTypeCheck::LIST && !node_[key].IsSequence()) {
    throw YAMLException("Entry key=" + Q(key) + " must be a list" + in_);
  }

  return YamlReader(node_[key], error_location_msg);
}

YamlReader YamlReader::SubNodeOpt(const std::string &key,
                                  NodeTypeCheck type_check,
                                  std::string error_location_msg) {
  if (!node_[key]) {
    return YamlReader(YAML::Node(), error_location_msg);
  }
  return SubNode(key, type_check, error_location_msg);
}

Vec2 YamlReader::GetVec2(const std::string &key) {
  std::vector<double> v = GetList<double>(key, 2, 2);
  return Vec2(v[0], v[1]);
}

Color YamlReader::GetColor(const std::string &key, const Color &default_val) {
  std::vector<double> v = GetList<double>(
      key, {default_val.r, default_val.g, default_val.b, default_val.a}, 4, 4);
  return Color(v[0], v[1], v[2], v[3]);
}

Pose YamlReader::GetPose(const std::string &key) {
  std::vector<double> v = GetList<double>(key, 3, 3);
  return Pose(v[0], v[1], v[2]);
}

Pose YamlReader::GetPose(const std::string &key, const Pose &default_val) {
  if (!node_[key]) {
    return default_val;
  }

  return GetPose(key);
}
}