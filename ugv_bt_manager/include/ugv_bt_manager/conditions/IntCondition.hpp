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
#include <std_msgs/Int32.h>
#include <string>


namespace BehaviorTreeNodes {

/**
 * @class IntCondition
 * @brief A Behavior Tree condition node that subscribes to an integer topic and checks its value.
 *
 * This node subscribes to a specified topic and checks the integer value received.
 * It returns SUCCESS if the value is 1 (connected), FAILURE if the value is 0 (not connected)
 * or 2 (low signal) after a timeout period.
 *
 * Ports:
 * - topic_in (std::string, Input): Name of the ROS topic to subscribe to.
 * - timeout_unconnected (int, Input): Timeout in seconds for unconnected state.
 * - timeout_low (int, Input): Timeout in seconds for low signal state.
 */
class IntCondition : public BT::ConditionNode {
public:
  std::string name;
  ros::Subscriber sub_int;
  std::string topic_in = "/internet_signal";
  int last_signal = 1;
  ros::Time last_change_time;
  int timeout_unconnected = 10;
  int timeout_low = 20;
  ros::NodeHandle public_node;

  bool init = false;

  IntCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("timeout_unconnected", "Timeout in seconds for unconnected state"),
      BT::InputPort<int>("timeout_low", "Timeout in seconds for low signal state"),
      BT::InputPort<std::string>("topic_in", "Input topic")
    };
  }

  BT::NodeStatus tick() override;
  void callbackIntCondition(const std_msgs::Int32& msg);
};

/**
  * @brief The main execution function of the node, called once per activation.
  * @return The status of the node (RUNNING, SUCCESS, or FAILURE).
  */
inline BT::NodeStatus IntCondition::tick() {
  setStatus(BT::NodeStatus::RUNNING);

  if (init) {
    auto res = BT::TreeNode::getInput("topic_in", topic_in);
    if (!res) {
      ROS_INFO("Input topic is empty. Check topic_in port.");
      return BT::NodeStatus::FAILURE;
    }

    sub_int = public_node.subscribe(topic_in, 5, &IntCondition::callbackIntCondition, this);
    init = false;
  }

  ros::spinOnce();
  ros::Time now = ros::Time::now();

  switch (last_signal) {
    case 1:  // connected
      last_change_time = now;
      return BT::NodeStatus::SUCCESS;
    case 0:  // not connected
    {
      double elapsed = (now - last_change_time).toSec();
      return (elapsed >= timeout_unconnected) ? BT::NodeStatus::FAILURE
                                              : BT::NodeStatus::SUCCESS;
    }
    case 2:  // low signal
    {
      double elapsed = (now - last_change_time).toSec();
      return (elapsed >= timeout_low) ? BT::NodeStatus::FAILURE
                                      : BT::NodeStatus::SUCCESS;
    }
    default:
      return BT::NodeStatus::FAILURE;
  }
}

IntCondition::IntCondition(const std::string& _name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(_name, config), name("[" + _name + "] ")
{
  ros::NodeHandle private_node("~");

  init = true;

  ROS_INFO("Condition initialized");
}

/**
 * @brief Callback function for processing incoming Int32 messages.
 *
 * This function updates the internal state when a new integer signal is received.
 * If the received value differs from the previous signal, it updates the last_signal
 * and records the time of change.
 *
 * @param msg The incoming std_msgs::Int32 message containing the new integer value.
 */
void IntCondition::callbackIntCondition(const std_msgs::Int32& msg) {
  if (msg.data != last_signal) {
    last_signal = msg.data;
    last_change_time = ros::Time::now();
  }
}

}  // namespace BehaviorTreeNodes
