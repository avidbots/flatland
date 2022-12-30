/*
 * @copyright Copyright 2022 FEUP
 * @name interactive_tool.hpp
 * @brief allows interaction with interactive markers
 * @author Ana Barros
 * @author Henrique Ribeiro
 * @author João Costa
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, FEUP
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

#ifndef FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
#define FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_

#include <vector>
#include <rviz_common/display.hpp>
#include <rviz_default_plugins/tools/interaction/interaction_tool.hpp>

namespace flatland_rviz_plugins
{
class RVIZ_DEFAULT_PLUGINS_PUBLIC InteractiveTool
  : public rviz_default_plugins::tools::InteractionTool
{
public:
  InteractiveTool();

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

private:
  void enableMarkers(bool is_enabled);
};
}  // namespace flatland_rviz_plugins

#endif  // FLATLAND_RVIZ_PLUGINS__INTERACTIVE_TOOL_HPP_
