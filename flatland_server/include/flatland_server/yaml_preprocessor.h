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
 * @param[in] ref_path The path the file was loaded from; used to locate
 * include files with relative filenames.
 * @return The parsed YAML::Node
 */
void Parse(YAML::Node &node, const std::string &ref_path);

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
 * @param[in] ref_path The path the file was loaded from; used to locate
 * include files with relative filenames.
 */
void ProcessNodes(YAML::Node &node, const std::string &ref_path);

/**
 * @brief Find and run any $eval expressions
 * @param[in/out] node A Yaml string node to parse
 */
void ProcessEvalNode(YAML::Node &node);

/**
 * @brief Resolve a given filename to an absolute path, resolving relative
 * filenames relative to ref_path
 * @param[in] filename The filename to find.
 * @param[in] ref_path Filename of the YAML file containing the `$include`, used
 * to locate relative filenames
 * @return The absolute path of the file to be included.
 */
std::string ResolveIncludeFilePath(const std::string &filename,
                                   const std::string &ref_path);

/**
 * @brief Potentially process an $include expression.
 * @param[in/out] node A Yaml string node to parse. If an include is processed,
 * the node is replaced with the contents of the specified file.
 * @param[in] ref_path The path the file was loaded from; used to locate
 * include files with relative filenames.
 */
void ProcessIncludeNode(YAML::Node &node, const std::string &ref_path);

/**
 * @brief Process a node, converting sequence include expression
 * ('$[include]')to a series of nodes.
 * @param[in/out] out_elems Vector that will be populated with parsed nodes
 * from the included file.
 * @param[in] node Node to parse. Should be a scalar node.
 * @param ref_path Reference path used for locating relative filenames.
 * @return If the node is an include expression, returns True. Otherwise,
 * False.
 *
 * If the function returns true, then `node` should be replaced by out_elems in
 * its sequence.
 */
bool ProcessSequenceIncludeNode(std::vector<YAML::Node> &out_elems,
                                YAML::Node &node, const std::string &ref_path);

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
