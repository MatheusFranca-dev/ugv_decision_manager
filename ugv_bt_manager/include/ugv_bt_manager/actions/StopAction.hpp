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

#include <string>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <ackermann_msgs/AckermannDrive.h>


namespace BehaviorTreeNodes {

/**
 * @class StopAction
 * @brief A Behavior Tree synchronous action node that publishes a zero-speed
 *        AckermannDrive message to stop the vehicle.
 *
 * This node is designed for use in a behavior tree controlling an UGV.
 * When ticked, it publishes an AckermannDrive message with all fields set to zero.
 * The topic to publish to is configurable via the "topic_out" input port.
 */
class StopAction : public BT::SyncActionNode {
public:
  /**
   * @brief Construct a new StopAction object
   * @param name The name of the node
   * @param config The node configuration
   */
  explicit StopAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
    ROS_INFO("StopAction: Node initialized");
  }

  /**
   * @brief Defines the ports that this node requires.
   * @return BT::PortsList The list of ports
   */
  static BT::PortsList providedPorts() {
    return { 
        BT::InputPort<std::string>("topic_out", "/gem/ackermann_cmd",
                                   "Topic to publish the stop command") 
    };
  }

  /**
   * @brief The main execution function of the node, called once per activation.
   * @return The status of the node (RUNNING, SUCCESS, or FAILURE).
   */
  BT::NodeStatus tick() override {
    // The first time tick() is called, we initialize the publisher.
    if (!pub_) {
      std::string topic_name;

      if (!getInput<std::string>("topic_out", topic_name)) {
        ROS_ERROR("StopAction: missing required input port 'topic_out'");
        return BT::NodeStatus::FAILURE;
      }

      // Create the ROS publisher for the AckermannDrive message.
      pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(topic_name, 1);
      ROS_INFO_STREAM("StopAction: Publishing stop commands to topic '" << topic_name << "'");
    }

    // Create the message to be published.
    ackermann_msgs::AckermannDrive stop_msg;
    stop_msg.speed = 0.0f;
    stop_msg.acceleration = 0.0f;
    stop_msg.jerk = 0.0f;
    stop_msg.steering_angle = 0.0f;
    stop_msg.steering_angle_velocity = 0.0f;

    pub_.publish(stop_msg);

    return BT::NodeStatus::SUCCESS;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher  pub_;
};

} // namespace BehaviorTreeNodes
