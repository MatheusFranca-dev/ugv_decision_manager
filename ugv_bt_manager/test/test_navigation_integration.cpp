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
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ugv_bt_interfaces/WaypointAction.h>

#include "../include/ugv_bt_manager/conditions/BoolCondition.hpp"
#include "../include/ugv_bt_manager/conditions/IntCondition.hpp"
#include "../include/ugv_bt_manager/conditions/FloatCondition.hpp"
#include "../include/ugv_bt_manager/actions/ActionWaypointClient.hpp"
#include "../include/ugv_bt_manager/actions/StopAction.hpp"


/**
 * @class FakeWaypointActionServer
 * @brief A mock implementation of an action server for waypoint goals, used for testing integration.
 *
 * This class simulates a ROS action server for waypoint navigation. It counts the number of goals received,
 * simulates processing each goal, and always reports success. Intended for use in integration tests where
 * a real waypoint action server is not required.
 */
class FakeWaypointActionServer {
public:
    FakeWaypointActionServer(const std::string& name) :
        as_(nh_, name, boost::bind(&FakeWaypointActionServer::executeCb, this, _1), false),
        action_name_(name),
        goal_received_count_(0)
    {
        as_.start();
        ROS_INFO_STREAM("FakeWaypointActionServer started on [" << name << "]");
    }

    /**
     * @brief Callback function executed when a new waypoint goal is received.
     *
     * Increments the count of received goals, logs the event, simulates processing
     * by sleeping for 0.2 seconds, and then sets the action server state to succeeded.
     *
     * @param goal The received waypoint goal (const pointer).
     */
    void executeCb(const ugv_bt_interfaces::WaypointGoalConstPtr& goal) {
        goal_received_count_++;
        ROS_INFO_STREAM("FakeWaypointActionServer received a goal! Total received: " << goal_received_count_);

        // Simulate work and then succeed
        ros::Duration(0.2).sleep();
        as_.setSucceeded();
    }

    /**
     * @brief Returns the number of goals that have been received.
     *
     * @return The count of received goals.
     */
    int getGoalReceivedCount() const {
        return goal_received_count_;
    }

    /**
     * @brief Resets the goal received count to zero.
     *
     * This method sets the internal counter for received goals back to 0.
     * It can be used to clear the state before starting a new navigation test.
     */
    void reset() {
        goal_received_count_ = 0;
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ugv_bt_interfaces::WaypointAction> as_;
    std::string action_name_;
    int goal_received_count_;
};



/**
 * @class NavigationIntegrationTest
 * @brief Integration test suite for navigation behavior tree in UGV Decision Manager.
 *
 * This test class sets up a ROS environment to validate the integration of custom Behavior Tree (BT) nodes
 * and the waypoint action server. It uses Google Test framework and provides utilities for publishing
 * condition topics and simulating navigation scenarios.
 *
 * Responsibilities:
 * - Registers custom BT nodes (BoolCondition, IntCondition, FloatCondition, ActionWaypointClient, StopAction).
 * - Initializes a fake waypoint action server for testing action client behavior.
 * - Publishes test data to condition topics (estop, internet_signal, battery_level, temperature, gps_accuracy, crosstrack_err).
 * - Provides a helper to publish all condition values at once.
 * - Loads a test BT XML configuration for navigation logic.
 */
class NavigationIntegrationTest : public ::testing::Test
{
protected:
    std::map<std::string, ros::Publisher> pubs_;
    static std::shared_ptr<FakeWaypointActionServer> fake_server_;
    static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
    static std::string waypoints_filepath_;

    /**
     * @brief Set up the test case for navigation integration tests.
     *
     * This static method initializes the BehaviorTreeFactory and registers all custom behavior tree nodes
     * required for the tests, including condition and action nodes. It also sets the file path for the waypoints
     * used in the test and starts a fake waypoint action server to simulate navigation actions.
     */
    static void SetUpTestCase() {
        factory_ = std::make_shared<BT::BehaviorTreeFactory>();

        // Register all the custom nodes used in the BT
        factory_->registerNodeType<BehaviorTreeNodes::BoolCondition>("BoolCondition");
        factory_->registerNodeType<BehaviorTreeNodes::IntCondition>("IntCondition");
        factory_->registerNodeType<BehaviorTreeNodes::FloatCondition>("FloatCondition");
        factory_->registerNodeType<BehaviorTreeNodes::ActionWaypointClient>("ActionWaypointClient");
        factory_->registerNodeType<BehaviorTreeNodes::StopAction>("StopAction");

        // Waypoints file for the test
        waypoints_filepath_ = "/config/short_waypoints.csv";

        // Start the fake action server
        fake_server_ = std::make_shared<FakeWaypointActionServer>("/waypoint_action");
    }

    /**
     * @brief Set up the test environment before each test case.
     *
     * This method resets the fake server's goal counter and initializes ROS publishers
     * for all required condition topics, including estop, internet signal, battery level,
     * temperature, GPS accuracy, and crosstrack error. It also allows time for the publishers
     * to establish connections before the test proceeds.
     */
    void SetUp() override {
        // Reset the server's goal counter before each test
        fake_server_->reset();

        // Create publishers for all condition topics
        ros::NodeHandle nh;
        pubs_["estop"] = nh.advertise<std_msgs::Bool>("estop_integration", 1, true);
        pubs_["internet_signal"] = nh.advertise<std_msgs::Int32>("internet_signal_integration", 1, true);
        pubs_["battery_level"] = nh.advertise<std_msgs::Float32>("battery_level_integration", 1, true);
        pubs_["temperature"] = nh.advertise<std_msgs::Float32>("temperature_integration", 1, true);
        pubs_["gps_accuracy"] = nh.advertise<std_msgs::Float32>("gps_accuracy_integration", 1, true);
        pubs_["crosstrack_err"] = nh.advertise<std_msgs::Float32>("crosstrack_err_integration", 1, true);

        // Allow time for publishers to connect
        ros::WallDuration(0.2).sleep();
    }

    /**
     * @brief Publishes diagnostic and status messages to their respective ROS topics.
     *
     * This function sends the provided values for emergency stop status, internet signal strength,
     * battery level, temperature, GPS accuracy, and crosstrack error to their corresponding publishers.
     * After publishing, it waits briefly to allow message processing and calls ros::spinOnce().
     *
     * @param estop Boolean indicating emergency stop status.
     * @param internet Integer representing internet signal strength.
     * @param battery Float representing battery level.
     * @param temp Float representing temperature.
     * @param gps Float representing GPS accuracy.
     * @param crosstrack Float representing crosstrack error.
     */
    void publish_all(bool estop, int internet, float battery, float temp, float gps, float crosstrack) {
        std_msgs::Bool b; b.data = estop;
        pubs_["estop"].publish(b);

        std_msgs::Int32 i; i.data = internet;
        pubs_["internet_signal"].publish(i);

        std_msgs::Float32 f;
        f.data = battery; pubs_["battery_level"].publish(f);
        f.data = temp; pubs_["temperature"].publish(f);
        f.data = gps; pubs_["gps_accuracy"].publish(f);
        f.data = crosstrack; pubs_["crosstrack_err"].publish(f);

        ros::WallDuration(0.1).sleep(); // Give a moment for messages to be processed
        ros::spinOnce();
    }

    // The BT XML
    const std::string bt_xml_ = R"(
        <root main_tree_to_execute="BehaviorTree">
          <BehaviorTree ID="BehaviorTree">
            <Sequence name="TopLevelSequence">
              <BoolCondition name="emergency_stop" topic_in="estop_integration"/>
              <ReactiveSequence name="NavigationChecks">
                <BoolCondition name="emergency_stop" topic_in="estop_integration"/>
                <IntCondition name="internet_connection" topic_in="internet_signal_integration" timeout_unconnected="10" timeout_low="20"/>
                <FloatCondition name="battery_level" topic_in="battery_level_integration" threshold="50" timeout="0" trigger_above="false"/>
                <FloatCondition name="temperature" topic_in="temperature_integration" threshold="55" timeout="0" trigger_above="true"/>
                <FloatCondition name="gps_accuracy" topic_in="gps_accuracy_integration" threshold="200" timeout="20" trigger_above="false"/>
                <Fallback>
                    <FloatCondition name="crosstrack_error" topic_in="crosstrack_err_integration" threshold="0.5" timeout="0" trigger_above="true"/>
                    <ForceFailure>
                        <StopAction name="stop_robot" topic_out="gem/ackermann_cmd"/>
                    </ForceFailure>
                </Fallback>
                <ActionWaypointClient
                    name="ProcessWaypoints"
                    action_name="/waypoint_action"
                    waypoints_file=")" + waypoints_filepath_ + R"("/>
              </ReactiveSequence>
            </Sequence>
          </BehaviorTree>
        </root>
    )";
};

// Initialize static members
std::shared_ptr<FakeWaypointActionServer> NavigationIntegrationTest::fake_server_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> NavigationIntegrationTest::factory_ = nullptr;
std::string NavigationIntegrationTest::waypoints_filepath_ = "";


// =================================================================================
// TEST SCENARIO 1: Should navigate when all conditions are safe
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNavigateWhenAllConditionsAreSafe) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing all safe conditions...");
    publish_all(false, 1, 80.0, 40.0, 300.0, 0.2);

    // Tick the tree until it completes
    auto status = tree.tickRoot();
    while(status == BT::NodeStatus::RUNNING) {
        ros::WallDuration(0.1).sleep();
        status = tree.tickRoot();
    }

    // The tree should succeed, and the action server should have been called once.
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 1);
}


// =================================================================================
// TEST SCENARIO 2: Should not navigate when Estop is active
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNotNavigateWhenEstopIsActive) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing active e-stop...");
    publish_all(true, 1, 80.0, 40.0, 300.0, 0.2); // estop is true

    auto status = tree.tickRoot();

    // The tree should fail immediately, and the action server should not be called.
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);
}


// =================================================================================
// TEST SCENARIO 3: Should not navigate when battery is low
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNotNavigateWhenBatteryIsLow) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing low battery...");
    publish_all(false, 1, 49.0, 40.0, 300.0, 0.2); // battery is 49%

    auto status = tree.tickRoot();

    // The tree should fail, and the action server should not be called.
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);
}


// =================================================================================
// TEST SCENARIO 4: Should not navigate when crosstrack error is high
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNotNavigateWhenCrosstrackErrorIsHigh) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing high crosstrack error...");
    publish_all(false, 1, 80.0, 40.0, 300.0, 0.6); // crosstrack is 0.6

    // The tree should fail, and the action server should not be called.
    auto status = tree.tickRoot();
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);
}


// =================================================================================
// TEST SCENARIO 5: Should not navigate when GPS is bad for too long
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNotNavigateWhenGpsIsBadForTooLong) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing bad GPS accuracy...");
    publish_all(false, 1, 80.0, 40.0, 150.0, 0.2); // gps accuracy is 150

    auto status = tree.tickRoot();
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);

    ROS_INFO("[TEST] Waiting for GPS timeout...");
    ros::WallDuration(20.0).sleep();

    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 1);
}


// =================================================================================
// TEST SCENARIO 6: Should not navigate when network is disconnected
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNotNavigateWhenNetworkIsDisconnected) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Publishing disconnected internet signal...");
    publish_all(false, 0, 80.0, 40.0, 300.0, 0.2); // internet is 0 (disconnected)

    auto status = tree.tickRoot();
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);

    ROS_INFO("[TEST] Waiting for internet timeout...");
    ros::WallDuration(10.0).sleep();

    EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 1);
}


// =================================================================================
// TEST SCENARIO 7: Should navigate after recovery from high temperature
// =================================================================================
TEST_F(NavigationIntegrationTest, ShouldNavigateAfterRecovery) {
    auto tree = factory_->createTreeFromText(bt_xml_);

    ROS_INFO("[TEST] Starting with high temperature...");
    publish_all(false, 1, 80.0, 60.0, 300.0, 0.2); // temp is 60 C
    auto status = tree.tickRoot();

    // Verify it fails and doesn't call the action
    EXPECT_EQ(status, BT::NodeStatus::FAILURE);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 0);

    ROS_INFO("[TEST] Temperature recovering...");
    publish_all(false, 1, 80.0, 50.0, 300.0, 0.2); // temp is now 50 C

    // Tick again until it completes
    status = tree.tickRoot();
    while(status == BT::NodeStatus::RUNNING) {
        ros::WallDuration(0.1).sleep();
        status = tree.tickRoot();
    }

    // Now it should succeed and have called the action server once.
    EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
    EXPECT_EQ(fake_server_->getGoalReceivedCount(), 1);
}


int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_navigation_integration");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int ret = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();

    return ret;
}
