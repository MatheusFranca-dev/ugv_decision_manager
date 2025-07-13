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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <std_msgs/Bool.h>

#include "../include/ugv_bt_manager/conditions/BoolCondition.hpp"

/**
 * @brief This test case is designed to verify the functionality of the BoolCondition BT node.
 *
 * Test Scenario: Emergency Stop
 * - A Behavior Tree is created with a single BoolCondition node.
 * - The node is configured to listen to the "/emergency_stop" topic.
 * - A `true` message is published to the topic.
 * - The test asserts that the BoolCondition node returns FAILURE, which is the expected
 * behavior for a condition that signals a stop or an error state.
 */
TEST(BoolConditionTest, EmergencyStopReturnsFailure) {
  BT::BehaviorTreeFactory factory;

  factory.registerNodeType<BehaviorTreeNodes::BoolCondition>("BoolCondition");
  const std::string topic_name = "/estop";

  // Define the Behavior Tree structure as an XML string
  std::string xml_text = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Condition ID="BoolCondition" topic_in=")" + topic_name + R"("/>
      </BehaviorTree>
    </root>
  )";

  // Create the Behavior Tree from the XML definition.
  auto tree = factory.createTreeFromText(xml_text);

  // Create a ROS NodeHandle and a Publisher to send messages to the node
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Bool>(topic_name, 1, true); // Latching publisher

  // Give the publisher a moment to establish connection
  ros::WallDuration(0.5).sleep();

  tree.tickRoot();

  std_msgs::Bool msg;
  msg.data = true;
  pub.publish(msg);

  // Allow time for the message to be processed.
  ros::WallDuration(0.1).sleep();
  ros::spinOnce();

  // Tick the tree again. Now that the node has received `true`, its status should be FAILURE.
  BT::NodeStatus status = tree.tickRoot();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_bool_condition");

  return RUN_ALL_TESTS();
}
