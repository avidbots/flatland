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

#include <Box2D/Box2D.h>
#include <flatland_server/exceptions.h>
#include <flatland_server/types.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include <boost/algorithm/string.hpp>
#include <boost/version.hpp>

// If we have a version of boost with type_index
#if BOOST_VERSION / 100 % 1000 >= 56
#include <boost/type_index.hpp>
#define TYPESTRING(T) (boost::typeindex::type_id<T>().pretty_name())
#else
#include <typeindex>
#include <typeinfo>
#define TYPESTRING(T) (typeid(T).name())
#endif

#include <set>
#include <string>
#include <vector>

namespace flatland_server {

/**
 */
class YamlReader {
 public:
  enum NodeTypeCheck { MAP, LIST, NO_CHECK };

  YAML::Node node_;                      ///< The YAML Node this processes
  std::set<std::string> accessed_keys_;  /// Records of the keys processed
  ///< location of the entry, used to show where the error come from
  std::string filename_;
  std::string file_path_;
  std::string entry_location_;
  ///< name of the yaml entry, used to show where the error come from
  std::string entry_name_;
  std::string fmt_in_;    ///< formatted entry location for display
  std::string fmt_name_;  ///< formatted entry location for display

  /**
   * @brief Default constructor for yaml reader, initialize with a empty yaml
   * Node
   */
  YamlReader();

  /**
   * @brief Constructor with a given node
   * @param[in] node A Yaml node to get data from
   */
  YamlReader(const YAML::Node &node);

  /**
   * @brief Constructor with a given path to a yaml file, throws exception on
   * failure
   * @param[in] path Path to the yaml file
   */
  YamlReader(const std::string &path);

  /**
   * @brief Use this method to set the entry location and entry name for error
   * message purposes
   * @param[in] entry_location Location of the entry, use "_NONE_" for no entry,
   * empty string indicates keep current value
   * @param[in] entry_name Name of the entry, optional, use "_NONE_" for no
   * entry, empty string indicates keep current value
   */
  void SetErrorInfo(std::string entry_location, std::string entry_name = "");

  /**
   * @brief Use this method to set the file location of the yaml node
   * @param[in] file_path path to the file, use "_NONE_" for empty string,
   * file_path with empty string will keep current path
   */
  void SetFile(const std::string &file_path);

  /**
   * @brief This method checks all keys in the yaml node are used, otherwise it
   * throws an exception
   */
  void EnsureAccessedAllKeys();

  /**
   * @return The yaml cpp Yaml Node
   */
  YAML::Node Node();

  /**
   * @return If the node is null, a.k.a if it is empty
   */
  bool IsNodeNull();

  /**
   * @return The number of subnodes in the node, a.k.a size of the node
   */
  int NodeSize();

  /**
   * @brief Get one of the subnode using a index, throws exception on failure,
   * file path is inherited from the parent
   * @param[in] index Index of the subnode
   * @param[in] type_check Check if the type of the subnode, i.e. list or map
   * @param[in] sub_node_location The error location of the subnode, optional,
   * if not provided or a given an empty string, it will generate the location
   * using its parents entry location and entry name. It also accepts "_NONE_"
   * @return YamlReader of the sub node
   */
  YamlReader Subnode(int index, NodeTypeCheck type_check,
                     std::string sub_node_location = "");
  /**
   * @brief Get one of the subnode using a key, throws exception on failure,
   * file path is inherited from the parent
   * @param[in] key Key of the subnode
   * @param[in] type_check Check if the type of the subnode, i.e. list or map
   * @param[in] sub_node_location The error location of the subnode, optional,
   * if not provided or a given an empty string, it will generate the location
   * using its parents entry location and entry name. It also accepts "_NONE_"
   * @return YamlReader of the sub node
   */
  YamlReader Subnode(const std::string &key, NodeTypeCheck type_check,
                     std::string sub_node_location = "");
  /**
   * @brief Optionally get one of the subnode using a key, throws exception on
   * failure, file path is inherited from the parent
   * @param[in] key Key of the subnode
   * @param[in] type_check Check if the type of the subnode, i.e. list or map
   * @param[in] sub_node_location The error location of the subnode, optional,
   * if not provided or a given an empty string, it will generate the location
   * using its parents entry location and entry name. It also accepts "_NONE_"
   * @return YamlReader of the sub node, or a YamlReader of a empty node if a
   * node with the given key does not exist
   */
  YamlReader SubnodeOpt(const std::string &key, NodeTypeCheck type_check,
                        std::string sub_node_location = "");
  /**
   * @brief Convert the current node in yaml reader to a given template type. It
   * uses yaml-cpp's as<T>() function, you could specify conversion for custom
   * datatypes using yaml cpp's documented ways, this method throws exception on
   * failure
   * @return The value after conversion.
   */
  template <typename T>
  T As();

  /**
   * @brief Convert the current node to a list of given type, throws exception
   * on failure
   * @return List of given type
   */
  template <typename T>
  std::vector<T> AsList(int min_size, int max_size);

  /**
   * @brief Convert the current node to a array of given type, throws exception
   * on failure
   * @return Array of given type
   */
  template <typename T, int N>
  std::array<T, N> AsArray();

  /**
   * @brief Get subnode with a given key and converted to the given type, throws
   * on failure
   * @param[in] key Key to access the subnode
   * @return Value of the converted subnode
   */
  template <typename T>
  T Get(const std::string &key);

  /**
   * @brief Optionally get subnode with a given key and converted to the given
   * type, throws on failure
   * @param[in] key Key to access the subnode
   * @param[in] default_val Default value
   * @return Value of the converted subnode, or default value if node with key
   * does not exist
   */
  template <typename T>
  T Get(const std::string &key, const T &default_val);

  /**
   * @brief Get subnode with a given key and converted to list of the given
   * type, throws on failure
   * @param[in] key Key to access the subnode
   * @param[in] min_size Minimum size of the list, -1 to ignore
   * @param[in] max_size Maximum size of the list, -1 to ignore
   * @return Value of the converted subnode
   */
  template <typename T>
  std::vector<T> GetList(const std::string &key, int min_size, int max_size);

  /**
   * @brief Optionally get subnode with a given key and converted to list of the
   * given type, throws on failure
   * @param[in] key Key to access the subnode
   * @param[in] default_val Default value
   * @param[in] min_size Minimum size of the list, -1 to ignore
   * @param[in] max_size Maximum size of the list, -1 to ignore
   * @return Value of the converted subnode
   */
  template <typename T>
  std::vector<T> GetList(const std::string &key,
                         const std::vector<T> default_val, int min_size,
                         int max_size);

  /**
   * @brief Get subnode with a given key and converted to array of the given
   * type, throws on failure
   * @param[in] key Key to access the subnode
   * @return Value of the converted subnode
   */
  template <typename T, int N>
  std::array<T, N> GetArray(const std::string &key);

  /**
   * @brief Optionally get subnode with a given key and converted to array of
   * a given type, throws on failure
   * @param[in] key Key to access the subnode
   * @param[in] default_val Default value
   * @return Value of the converted subnode
   */
  template <typename T, int N>
  std::array<T, N> GetArray(const std::string &key,
                            const std::array<T, N> default_val);

  /**
   * @return A Vec2 accessed by a given key
   */
  Vec2 GetVec2(const std::string &key);

  /**
   * @return A Vec2 accessed by a given key, or default value if key does not
   * exist
   */
  Vec2 GetVec2(const std::string &key, const Vec2 &default_val);

  /**
   * @return A Color value accessed by a given key, or default value if key does
   * not exist
   */
  Color GetColor(const std::string &key, const Color &default_val);

  /**
   * @return A Pose value accessed by a given key
   */
  Pose GetPose(const std::string &key);

  /**
   * @return A Pose value accessed by a given key, or default value if key does
   * not exist
   */
  Pose GetPose(const std::string &key, const Pose &default_val);
};

/**
 * @return A string with quotes around the input
 */
inline std::string Q(const std::string &str) { return "\"" + str + "\""; }

template <typename T>
T YamlReader::As() {
  T ret;

  try {
    ret = node_.as<T>();
  } catch (const YAML::RepresentationException &e) {
    throw YAMLException("Error converting entry" + fmt_name_ + " to " +
                        TYPESTRING(T) + fmt_in_);
  } catch (const YAML::Exception &e) {
    throw YAMLException("Error reading entry" + fmt_name_ + fmt_in_);
  }

  return ret;
}

template <typename T>
std::vector<T> YamlReader::AsList(int min_size, int max_size) {
  std::vector<T> list;

  if (min_size > 0 && max_size > 0 && min_size == max_size &&
      NodeSize() != max_size) {
    throw YAMLException("Entry" + fmt_name_ + " must have size of exactly " +
                        std::to_string(min_size) + fmt_in_);
  }

  if (min_size > 0 && NodeSize() < min_size) {
    throw YAMLException("Entry" + fmt_name_ + " must have size >= " +
                        std::to_string(min_size) + fmt_in_);
  }

  if (max_size > 0 && NodeSize() > max_size) {
    throw YAMLException("Entry" + fmt_name_ + " must have size <= " +
                        std::to_string(max_size) + fmt_in_);
  }

  for (int i = 0; i < NodeSize(); i++) {
    list.push_back(Subnode(i, NO_CHECK).As<T>());
  }

  return list;
}

template <typename T, int N>
std::array<T, N> YamlReader::AsArray() {
  std::vector<T> list_ret = AsList<T>(N, N);
  std::array<T, N> array_ret;

  for (int i = 0; i < N; i++) {
    array_ret[i] = list_ret[i];
  }
  return array_ret;
}

template <typename T>
T YamlReader::Get(const std::string &key) {
  return Subnode(key, NO_CHECK).As<T>();
}

template <typename T>
T YamlReader::Get(const std::string &key, const T &default_val) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return default_val;
  }
  return Get<T>(key);
}

template <typename T>
std::vector<T> YamlReader::GetList(const std::string &key, int min_size,
                                   int max_size) {
  return Subnode(key, LIST).AsList<T>(min_size, max_size);
}

template <typename T>
std::vector<T> YamlReader::GetList(const std::string &key,
                                   const std::vector<T> default_val,
                                   int min_size, int max_size) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return default_val;
  }

  return GetList<T>(key, min_size, max_size);
}

template <typename T, int N>
std::array<T, N> YamlReader::GetArray(const std::string &key) {
  return Subnode(key, LIST).AsArray<T, N>();
}

template <typename T, int N>
std::array<T, N> YamlReader::GetArray(const std::string &key,
                                      const std::array<T, N> default_val) {
  if (!node_[key]) {
    accessed_keys_.insert(key);
    return default_val;
  }

  return GetArray<T, N>(key);
}
}

// encode and decode functions for yaml-cpp to convert values for commonly used
// types in flatland server
namespace YAML {
template <>
struct convert<b2Vec2> {
  static bool decode(const Node &node, b2Vec2 &rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Vec2> {
  static bool decode(const Node &node, flatland_server::Vec2 &rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Color> {
  static bool decode(const Node &node, flatland_server::Color &rhs) {
    if (!node.IsSequence() || node.size() != 4) {
      return false;
    }

    rhs.r = node[0].as<double>();
    rhs.g = node[1].as<double>();
    rhs.b = node[2].as<double>();
    rhs.a = node[3].as<double>();
    return true;
  }
};

template <>
struct convert<flatland_server::Pose> {
  static bool decode(const Node &node, flatland_server::Pose &rhs) {
    if (!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.theta = node[2].as<double>();
    return true;
  }
};
}

#endif
