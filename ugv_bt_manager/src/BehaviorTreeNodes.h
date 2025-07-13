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

#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <ugv_bt_manager/actions/ActionWaypointClient.hpp>
#include <ugv_bt_manager/actions/StopAction.hpp>

#include <ugv_bt_manager/conditions/BoolCondition.hpp>
#include <ugv_bt_manager/conditions/IntCondition.hpp>
#include <ugv_bt_manager/conditions/FloatCondition.hpp>


namespace BehaviorTreeNodes {

/**
 * @brief Registers custom behavior tree nodes with the provided factory.
 *
 * This function adds action and condition node types to the BehaviorTreeFactory,
 * enabling their use in behavior tree construction. Specifically, it registers:
 * - Action nodes: ActionWaypointClient, StopAction
 * - Condition nodes: BoolCondition, IntCondition, FloatCondition
 *
 * @param factory Reference to the BehaviorTreeFactory where nodes will be registered.
 */
inline void RegisterNodes(BT::BehaviorTreeFactory& factory) {
  // register actions
  factory.registerNodeType<ActionWaypointClient>("ActionWaypointClient");
  factory.registerNodeType<StopAction>("StopAction");

  // register conditions
  factory.registerNodeType<BoolCondition>("BoolCondition");
  factory.registerNodeType<IntCondition>("IntCondition");
  factory.registerNodeType<FloatCondition>("FloatCondition");
}

}  // namespace BehaviorTreeNodes
