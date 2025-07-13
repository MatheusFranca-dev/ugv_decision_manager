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
#include <ackermann_msgs/AckermannDrive.h>
#include <boost/optional.hpp>

#include "../include/ugv_bt_manager/actions/StopAction.hpp"

// A global variable to store the received message for inspection
boost::optional<ackermann_msgs::AckermannDrive> received_msg;

/**
 * @brief Callback function for the ROS subscriber.
 * It's called whenever a message is published on the subscribed topic.
 * @param msg The received AckermannDrive message.
 */
void stopCommandCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
    received_msg = *msg;
    ROS_INFO("Test subscriber received a stop command.");
}


/**
 * @brief This test verifies that the StopAction node correctly publishes a
 * zero-value AckermannDrive message to the specified topic.
 */
TEST(StopActionTest, PublishesZeroCommandOnSuccess)
{
    ros::NodeHandle nh;
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BehaviorTreeNodes::StopAction>("StopAction");

    const std::string topic_name = "gem/ackermann_cmd";

    ros::Subscriber sub = nh.subscribe(topic_name, 1, stopCommandCallback);

    // Reset the global variable before the test
    received_msg.reset();

    // Define the Behavior Tree structure.
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <StopAction name="stop_robot" topic_out=")" + topic_name + R"("/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);

    // Allow a moment for the subscriber to connect
    ros::WallDuration(0.2).sleep();

    const auto status = tree.tickRoot();

    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // The node should return SUCCESS.
    ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

    // The subscriber should have received a message.
    ASSERT_TRUE(received_msg) << "Subscriber did not receive any message on topic: " << topic_name;

    // Verify that all fields in the received message are zero.
    if (received_msg) {
        EXPECT_FLOAT_EQ(received_msg->speed, 0.0f);
        EXPECT_FLOAT_EQ(received_msg->acceleration, 0.0f);
        EXPECT_FLOAT_EQ(received_msg->jerk, 0.0f);
        EXPECT_FLOAT_EQ(received_msg->steering_angle, 0.0f);
        EXPECT_FLOAT_EQ(received_msg->steering_angle_velocity, 0.0f);
    }
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_stop_action");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return ret;
}
