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

#ifndef FLATLAND_SERVER_YAML_READER_H
#define FLATLAND_SERVER_YAML_READER_H

#include <flatland_server/types.h>
#include <yaml-cpp/yaml.h>

namespace flatland_server {

/**
 */
class YamlReader {
 public:
  enum NodeTypeCheck { MAP, LIST, NO_CHECK };

  YAML::Node node_;                           ///< The YAML Node this processes
  std::map<std::string, bool> key_accessed_;  /// Records of the keys processed

  /**
   * @brief
   */
  YamlReader(const YAML::Node &node);
  YamlReader(const std::string &path);

  YAML::Node Node();

  bool IsNodeNull();
  int NodeSize();

  YamlReader SubNode(int index, NodeTypeCheck type_check, std::string in = "");
  YamlReader SubNode(const std::string &key, NodeTypeCheck type_check,
                     std::string in = "");
  YamlReader SubNodeOpt(const std::string &key, NodeTypeCheck type_check,
                        std::string in = "");

  template <typename T>
  T Get(const std::string &key, const std::string &type_name, std::string in);

  template <typename T>
  T GetOpt(const std::string &key, const T &default_val,
           const std::string &type_name, std::string in);

  //   template <typename T>
  //   std::vector<T> GetList(const std::string &key, const std::string
  //   &type_name,
  //                          int exact_size, int min_size, int max_size,
  //                          std::string in);

  //   template <typename T>
  //   T Index(const YAML::Node &node, int index, const ::string &type_name,
  //           std::string in);

  //   template <typename T>
  //   std::vector<T> GetListOpt(const std::string &key,
  //                             const std::vector<T> default_val, int
  //                             exact_size,
  //                             int min_size, int max_size,
  //                             const std::string &type_name, std::string in);

  double GetDouble(const std::string &key, std::string in = "");

  double GetDoubleOpt(const std::string &key, double default_val,
                      std::string in = "");

  std::string GetString(const std::string &key, std::string in = "");

  std::string GetStringOpt(const std::string &key,
                           const std::string &default_val, std::string in = "");

  Color GetColorOpt(const std::string &key, const Color &default_val,
                    std::string in = "");

  Pose GetPose(const std::string &key, std::string in = "");
  Pose GetPoseOpt(const std::string &key, const Pose &default_val,
                  std::string in = "");
};
};

#endif