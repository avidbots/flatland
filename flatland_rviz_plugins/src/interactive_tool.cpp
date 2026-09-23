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

#include "flatland_rviz_plugins/interactive_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_common/view_manager.hpp>

namespace flatland_rviz_plugins
{

InteractiveTool::InteractiveTool()
: rviz_default_plugins::tools::InteractionTool() {}

void InteractiveTool::enableMarkers(bool is_enabled)
{
  auto root_display = context_->getRootDisplayGroup();
  for (int i = 0; i < root_display->numDisplays(); ++i) {
    auto display = root_display->getDisplayAt(i);
    if (display->getClassId() == "rviz_default_plugins/InteractiveMarkers") {
      display->setEnabled(is_enabled);
    }
  }
}

void InteractiveTool::onInitialize()
{
  rviz_default_plugins::tools::InteractionTool::onInitialize();

  this->enableMarkers(false);

  setName("Interact");
  setIcon(rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/Interact.png"));
}

void InteractiveTool::activate()
{
  rviz_default_plugins::tools::InteractionTool::activate();

  this->enableMarkers(true);
}

void InteractiveTool::deactivate()
{
  rviz_default_plugins::tools::InteractionTool::deactivate();

  this->enableMarkers(false);
}

}  // namespace flatland_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(flatland_rviz_plugins::InteractiveTool, rviz_common::Tool)
