// Copyright 2022 Fraunhofer FKIE
// Copyright 2025 Matheus França (modified for UGV Decision Manager)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "BehaviorTreeManager.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "behavior_tree_manager");

  // Initializes ROS NodeHandles
  ros::NodeHandle public_node = ros::NodeHandle("");
  ros::NodeHandle private_node = ros::NodeHandle("~");

  BehaviorTreeManager bt_manager;

  // Update ros parameters
  bt_manager.params.readParameters();

  // Register available action plugins
  ROS_INFO("Registered plugins:");
  for (std::string ap : bt_manager.params.action_plugins)
    ROS_INFO("  - %s", ap.c_str());

  bt_manager.registerActionPlugins();
  bt_manager.reportBTState();

  // Initialize static tree
  if (!bt_manager.params.tree_static_xml.empty()) {
    bt_manager.initializeTreeFromText(bt_manager.params.tree_static_xml);
  }
  else if (!bt_manager.params.tree_static_file.empty()) {
    bt_manager.initializeTreeFromFile(bt_manager.params.tree_static_file);
  }
  else {
    ROS_INFO("No tree is available. Check parameters [tree/static_xml] or [tree/static_file]");
    return 0;
  }

  // Initialize loggin system
  bt_manager.initializeLoggers();

  bt_manager.spin();

  return 0;
}