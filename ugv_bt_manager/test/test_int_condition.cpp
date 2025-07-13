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
#include <std_msgs/Int32.h>

#include "../include/ugv_bt_manager/conditions/IntCondition.hpp"

/**
 * @brief Helper function to publish a message and process callbacks.
 * @param pub The ROS publisher.
 * @param signal The integer signal value to publish.
 */
void publish_and_spin(ros::Publisher& pub, int signal) {
    std_msgs::Int32 msg;
    msg.data = signal;
    pub.publish(msg);

    // Short sleep to allow message propagation and processing
    ros::WallDuration(0.1).sleep();
    ros::spinOnce();
}


/**
 * @brief This test case simulates network signal fluctuations to test the IntCondition node.
 *
 * Test Scenario:
 * 1. Starts with a stable connection (signal 1 -> SUCCESS).
 * 2. Signal drops to "low" (signal 2). The node should SUCCEED for a while.
 * 3. Signal returns to stable (signal 1 -> SUCCESS).
 * 4. Signal drops to "unconnected" (signal 0). The node should SUCCEED for a short
 * period (less than timeout_unconnected) and then FAIL.
 * 5. Signal returns to stable, and the node should recover to SUCCESS.
 */
TEST(IntConditionTest, NetworkSignalFluctuation) {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BehaviorTreeNodes::IntCondition>("IntCondition");

    const std::string topic_name = "/network_signal";
    const int timeout_unconnected = 10;
    const int timeout_low = 20;

    // Define the Behavior Tree, setting the timeout ports from the test.
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <Condition ID="IntCondition"
                           topic_in=")" + topic_name + R"("
                           timeout_unconnected=")" + std::to_string(timeout_unconnected) + R"("
                           timeout_low=")" + std::to_string(timeout_low) + R"("/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Int32>(topic_name, 1, true); // Latching

    // Wait for publisher to be ready
    ros::WallDuration(0.5).sleep();

    ROS_INFO_STREAM("[TEST] Setting initial state to Connected (1)");
    publish_and_spin(pub, 1);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[TEST] Switching to Low Signal (2)");
    publish_and_spin(pub, 2);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS); // Should succeed immediately after change

    ROS_INFO_STREAM("[TEST] Waiting 20 seconds with Low Signal...");
    ros::WallDuration(20.0).sleep();
    ros::spinOnce();
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[TEST] Returning to Connected (1)");
    publish_and_spin(pub, 1);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[TEST] Switching to Unconnected (0)");
    publish_and_spin(pub, 0);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS); // Should succeed immediately after change

    ROS_INFO_STREAM("[TEST] Waiting 10 seconds with Unconnected signal...");
    ros::WallDuration(10.0).sleep();
    ros::spinOnce();
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[TEST] Recovering connection to Connected (1)");
    publish_and_spin(pub, 1);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

// Main function for Google Test
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_int_condition");

    return RUN_ALL_TESTS();
}
