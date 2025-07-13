// MIT License

// Copyright (c) 2025 Matheus França

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


namespace BehaviorTreeNodes {

/**
 * @class BoolCondition
 * @brief A Behavior Tree condition node that subscribes to a boolean topic and checks its value.
 *
 * This node subscribes to a specified topic and checks the boolean value received.
 * It returns SUCCESS if the value is true, and FAILURE if the value is false.
 * The topic to subscribe to is configurable via the "topic_in" input port.
 */
class BoolCondition : public BT::ConditionNode {
public:
  std::string name;
  ros::Subscriber sub_bool;
  std::string topic_in;
  bool init = true;
  bool message_received = false;
  bool result_action = true;
  ros::NodeHandle public_node;

  BoolCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    BT::PortsList ports = { BT::InputPort<std::string>("topic_in", "Input topic") };
    return ports;
  }

  BT::NodeStatus tick() override;
  void callbackBool(const std_msgs::Bool& msg);
};

/**
  * @brief The main execution function of the node, called once per activation.
  * @return The status of the node (RUNNING, SUCCESS, or FAILURE).
  */
inline BT::NodeStatus BoolCondition::tick(){
  setStatus(BT::NodeStatus::RUNNING);

  if (init) {
    auto res = BT::TreeNode::getInput("topic_in", topic_in);
    if (!res) {
      ROS_INFO("Input topic is empty. Check topic_in port.");
      return BT::NodeStatus::FAILURE;
    }

    sub_bool = public_node.subscribe(topic_in, 5, &BoolCondition::callbackBool, this);
    init = false;
  }

  ros::spinOnce();

  if (!result_action)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

BoolCondition::BoolCondition(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(_name, config), name("[" + _name + "] ") {
  ros::NodeHandle private_node("~");

  init = true;

  ROS_INFO("Condition initialized");
}

/**
 * @brief Callback function to handle incoming boolean messages.
 *
 * This function is triggered when a new std_msgs::Bool message is received.
 * It sets the message_received flag to true and updates the result_action
 * with the value from the received message.
 *
 * @param msg The received std_msgs::Bool message containing the boolean data.
 */
void BoolCondition::callbackBool(const std_msgs::Bool& msg) {
  message_received = true;
  result_action = msg.data;
}

}  // namespace BehaviorTreeNodes
