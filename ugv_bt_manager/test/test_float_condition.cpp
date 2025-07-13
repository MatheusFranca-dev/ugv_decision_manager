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
#include <std_msgs/Float32.h>

#include "../include/ugv_bt_manager/conditions/FloatCondition.hpp"

/**
 * @brief Helper function to publish a message and process callbacks.
 * @param pub The ROS publisher.
 * @param value The float value to publish.
 */
void publish_float(ros::Publisher& pub, float value) {
    std_msgs::Float32 msg;
    msg.data = value;
    pub.publish(msg);

    // Short sleep to allow message propagation and processing
    ros::WallDuration(0.1).sleep();
}

/**
 * @brief Test fixture to set up the BT factory once for all FloatCondition tests.
 */
class FloatConditionTest : public ::testing::Test {
protected:
    static BT::BehaviorTreeFactory factory;

    static void SetUpTestCase() {
        // Register the node type once for all tests in this fixture
        factory.registerNodeType<BehaviorTreeNodes::FloatCondition>("FloatCondition");
    }
};
BT::BehaviorTreeFactory FloatConditionTest::factory;


// =================================================================================
// TEST SCENARIO 1: Battery Failure (trigger_above=false, timeout=0)
// =================================================================================
TEST_F(FloatConditionTest, BatteryFailure) {
    const std::string topic_name = "/battery_level";
    const float threshold = 50.0f;
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <FloatCondition name="battery_check"
                                topic_in=")" + topic_name + R"("
                                threshold=")" + std::to_string(threshold) + R"("
                                timeout="0"
                                trigger_above="false"/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>(topic_name, 1, true);
    ros::WallDuration(0.2).sleep(); // Wait for publisher

    ROS_INFO_STREAM("[BATTERY_TEST] Initial state: 100%");
    publish_float(pub, 100.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[BATTERY_TEST] Dropping to 51%");
    publish_float(pub, 51.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[BATTERY_TEST] Dropping to 50% (at threshold)");
    publish_float(pub, 50.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[BATTERY_TEST] Dropping to 49% (below threshold)");
    publish_float(pub, 49.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[BATTERY_TEST] Recovering to 50.1%");
    publish_float(pub, 50.1f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}


// =================================================================================
// TEST SCENARIO 2: Temperature Spike (trigger_above=true, timeout=0)
// =================================================================================
TEST_F(FloatConditionTest, TemperatureSpike) {
    const std::string topic_name = "/temperature";
    const float threshold = 55.0f;
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <FloatCondition name="temp_check"
                                topic_in=")" + topic_name + R"("
                                threshold=")" + std::to_string(threshold) + R"("
                                timeout="0"
                                trigger_above="true"/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>(topic_name, 1, true);
    ros::WallDuration(0.2).sleep();

    ROS_INFO_STREAM("[TEMP_TEST] Initial state: 30 C");
    publish_float(pub, 30.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[TEMP_TEST] Rising to 54.9 C");
    publish_float(pub, 54.9f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[TEMP_TEST] Rising to 55 C (at threshold)");
    publish_float(pub, 55.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[TEMP_TEST] Rising to 60 C (above threshold)");
    publish_float(pub, 60.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[TEMP_TEST] Cooling to 54.9 C");
    publish_float(pub, 54.9f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}


// =================================================================================
// TEST SCENARIO 3: GPS Fluctuation (trigger_above=false, timeout > 0)
// =================================================================================
TEST_F(FloatConditionTest, GpsFluctuation) {
    const std::string topic_name = "/gps_accuracy";
    const float threshold = 200.0f;
    const float timeout = 15.0f;
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <FloatCondition name="gps_check"
                                topic_in=")" + topic_name + R"("
                                threshold=")" + std::to_string(threshold) + R"("
                                timeout=")" + std::to_string(timeout) + R"("
                                trigger_above="false"/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>(topic_name, 1, true);
    ros::WallDuration(0.2).sleep();

    ROS_INFO_STREAM("[GPS_TEST] Initial state: High accuracy (300mm)");
    publish_float(pub, 300.0f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[GPS_TEST] Accuracy drops to 190mm (violation starts)");
    publish_float(pub, 190.0f);
    // NOTE: The timer starts now, but since elapsed (0s) < timeout (15s), it's SUCCESS.
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[GPS_TEST] Waiting 15 seconds...");
    ros::WallDuration(15.0).sleep();
    
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[GPS_TEST] Accuracy improves to 210mm (violation ends)");
    publish_float(pub, 210.0f);
    // As soon as the value is safe again, it should return to SUCCESS.
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}


// =================================================================================
// TEST SCENARIO 4: Crosstrack Error (trigger_above=true, timeout=0)
// =================================================================================
TEST_F(FloatConditionTest, CrosstrackError) {
    const std::string topic_name = "/crosstrack_err";
    const float threshold = 0.5f;
    std::string xml_text = R"(
        <root main_tree_to_execute="MainTree">
            <BehaviorTree ID="MainTree">
                <FloatCondition name="crosstrack_error"
                                topic_in=")" + topic_name + R"("
                                threshold=")" + std::to_string(threshold) + R"("
                                timeout="0"
                                trigger_above="true"/>
            </BehaviorTree>
        </root>
    )";

    auto tree = factory.createTreeFromText(xml_text);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float32>(topic_name, 1, true);
    ros::WallDuration(0.2).sleep();

    ROS_INFO_STREAM("[CROSSTRACK_TEST] Initial state: Low error (0.1)");
    publish_float(pub, 0.1f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[CROSSTRACK_TEST] Error increasing to 0.49");
    publish_float(pub, 0.49f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

    ROS_INFO_STREAM("[CROSSTRACK_TEST] Error at threshold (0.5)");
    publish_float(pub, 0.5f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[CROSSTRACK_TEST] Error above threshold (0.6)");
    publish_float(pub, 0.6f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);

    ROS_INFO_STREAM("[CROSSTRACK_TEST] Error recovering to 0.4");
    publish_float(pub, 0.4f);
    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}


// Main function for Google Test
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_float_condition");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return ret;
}
