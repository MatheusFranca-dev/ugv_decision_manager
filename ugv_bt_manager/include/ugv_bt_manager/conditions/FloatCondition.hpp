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
#include <std_msgs/Float32.h>
#include <string>
#include <behaviortree_cpp_v3/condition_node.h>
#include <ros/ros.h>


namespace BehaviorTreeNodes {

/**
 * @brief Condition node for evaluating a float value from a ROS topic against a threshold.
 *
 * This class subscribes to a specified ROS topic publishing std_msgs::Float32 messages.
 * It checks whether the received value satisfies a threshold condition, with configurable
 * timeout and trigger direction.
 *
 * Ports:
 * - topic_in (std::string, Input): Name of the ROS topic to subscribe to.
 * - threshold (float, Input): Threshold value for comparison.
 * - timeout (float, Input): Timeout in seconds. If <= 0, check is instant; if > 0, elapsed-time check.
 * - trigger_above (bool, Input): If true, condition fails when value ≥ threshold; if false, fails when value ≤ threshold.
 *
 * The node resets its timer whenever a new value is received. The tick() method evaluates
 * the condition based on the latest value and the provided configuration.
 */
class FloatCondition : public BT::ConditionNode {
public:
  explicit FloatCondition(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("topic_in", "Input topic"),
      BT::InputPort<float>("threshold", "Threshold value"),
      BT::InputPort<float>("timeout", "Seconds: <=0 = instant, >0 = elapsed-time check"),
      BT::InputPort<bool>("trigger_above",
                          "True = FAIL when value ≥ threshold | False = FAIL when value ≤ threshold")
    };
  }

  BT::NodeStatus tick() override;

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::string topic_in_;
  float last_value_{0.0f};
  ros::Time last_change_time_{ros::Time::now()};
  bool init_{true};

  /**
   * @brief Callback function to handle incoming Float32 messages.
   *
   * This function updates the internal state with the latest value received
   * from a std_msgs::Float32 message and records the time of the change.
   *
   * @param msg The received Float32 message containing the new value.
   */
  void callback(const std_msgs::Float32& msg) {
    last_value_ = msg.data;
    last_change_time_ = ros::Time::now();
  }
};

inline FloatCondition::FloatCondition(const std::string& name,
                                      const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config) {
  ROS_INFO("Condition initialized");
}

/**
  * @brief The main execution function of the node, called once per activation.
  * @return The status of the node (RUNNING, SUCCESS, or FAILURE).
  */
inline BT::NodeStatus FloatCondition::tick()
{
  if (init_)
  {
    // Grab parameters
    auto ok = getInput("topic_in", topic_in_);
    if (!ok) {
      ROS_INFO("FloatCondition: missing port 'topic_in'");
      return BT::NodeStatus::FAILURE;
    }
    sub_ = nh_.subscribe(topic_in_, 5, &FloatCondition::callback, this);
    init_ = false;
  }

  // Process any new message
  ros::spinOnce();
  const ros::Time now = ros::Time::now();

  // Read the ports
  float threshold{0.0f}, timeout{0.0f};
  bool  trigger_above{false};
  if (!getInput("threshold", threshold)) {
    ROS_INFO("FloatCondition: missing port 'threshold'");
    return BT::NodeStatus::FAILURE;
  }
  getInput("timeout", timeout);  
  getInput("trigger_above", trigger_above);

  // Decide if we're in violation
  bool violation = trigger_above
                   ? (last_value_ >= threshold)
                   : (last_value_ <= threshold);

  // Immediate or timed check
  if (timeout <= 0.0f) {
    return violation ? BT::NodeStatus::FAILURE
                     : BT::NodeStatus::SUCCESS;
  }
  else {
    if (!violation) {
      last_change_time_ = now;
      return BT::NodeStatus::SUCCESS;
    }
    else {
      // still out‐of‐bounds: compare elapsed
      double elapsed = (now - last_change_time_).toSec();
      return (elapsed > timeout)
             ? BT::NodeStatus::FAILURE
             : BT::NodeStatus::SUCCESS;
    }
  }
}

}  // namespace BehaviorTreeNodes
