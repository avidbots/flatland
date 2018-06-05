/*
 *  ______                   __  __              __
 * /\  _  \           __    /\ \/\ \            /\ \__
 * \ \ \L\ \  __  __ /\_\   \_\ \ \ \____    ___\ \ ,_\   ____
 *  \ \  __ \/\ \/\ \\/\ \  /'_` \ \ '__`\  / __`\ \ \/  /',__\
 *   \ \ \/\ \ \ \_/ |\ \ \/\ \L\ \ \ \L\ \/\ \L\ \ \ \_/\__, `\
 *    \ \_\ \_\ \___/  \ \_\ \___,_\ \_,__/\ \____/\ \__\/\____/
 *     \/_/\/_/\/__/    \/_/\/__,_ /\/___/  \/___/  \/__/\/___/
 * @copyright Copyright 2018 Avidbots Corp.
 * @name	 yaml_preprocessor.h
 * @brief	 Yaml preprocessor using Lua
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

#ifndef FLATLAND_SERVER_YAML_PREPROCESSOR_H
#define FLATLAND_SERVER_YAML_PREPROCESSOR_H

#include <flatland_server/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
#include <set>
#include <string>
#include <vector>

namespace flatland_server {

/**
 */
namespace YamlPreprocessor {
/**
 * @brief Preprocess with a given node
 * @param[in/out] node A Yaml node to parse
 * @return The parsed YAML::Node
 */
void Parse(YAML::Node &node);

/**
 * @brief Constructor with a given path to a yaml file, throws exception on
 * failure
 * @param[in] path Path to the yaml file
 * @return The parsed YAML::Node
 */
YAML::Node LoadParse(const std::string &path);

/**
 * @brief Find and run any $eval nodes
 * @param[in/out] node A Yaml node to recursively parse
 */
void ProcessNodes(YAML::Node &node);

/**
 * @brief Find and run any $eval expressions
 * @param[in/out] node A Yaml string node to parse
 */
void ProcessScalarNode(YAML::Node &node);

/**
  * @brief Get an environment variable with an optional default value
  * @param[in/out] lua_State The lua state/stack to read/write to/from
  */
int LuaGetEnv(lua_State *L);

/**
  * @brief Get a rosparam with an optional default value
  * @param[in/out] lua_State The lua state/stack to read/write to/from
  */
int LuaGetParam(lua_State *L);
};
}

#endif  // FLATLAND_SERVER_YAML_PREPROCESSOR_H
