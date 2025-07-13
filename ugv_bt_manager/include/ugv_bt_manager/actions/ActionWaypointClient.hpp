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

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/client/simple_action_client.h>
#include <ugv_bt_interfaces/WaypointAction.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <tuple>
#include <memory>
#include <atomic>


namespace BehaviorTreeNodes {

/**
 * @class ActionWaypointClient
 * @brief Behavior Tree action node for sending waypoint navigation goals via ROS actionlib.
 *
 * This node loads a list of waypoints from a CSV file and sends them as a goal to a ROS action server.
 * It supports configuration via Behavior Tree XML input ports, and handles asynchronous
 * execution using BT::CoroActionNode. The node waits for the action server to become available, sends the
 * goal, and monitors its completion. If the action server is not available or waypoints cannot be loaded,
 * the node is marked as invalid and will return FAILURE when ticked.
 *
 * Input Ports:
 *   - action_name: Name of the ROS action server (default: "/waypoint_action").
 *   - waypoints_file: Path to the waypoints CSV file relative to the package (default: "config/waypoints.csv").
 *
 * Usage:
 *   - Integrate this node into a Behavior Tree to enable waypoint-based navigation.
 *   - Configure the action server name and waypoints file via XML.
 */
class ActionWaypointClient : public BT::CoroActionNode {
public:
  /**
   * @brief Constructor for the ActionWaypointClient node.
   * @param name The name of the node.
   * @param config The configuration of the node.
   */
  ActionWaypointClient(const std::string& name, const BT::NodeConfiguration& config)
    // Call the base class constructor
    : BT::CoroActionNode(name, config)
    , name_{"[" + name + "] "}
    , valid_{false}
  {
    ros::NodeHandle pnh("~");

    // Get configuration for the node
    if (!getInput<std::string>("action_name", action_name_)) {
      ROS_INFO("Action name is empty. Check action_name port in xml.");
      return;
    }
    if (!getInput<std::string>("waypoints_file", waypoints_file_)) {
      ROS_INFO("Waypoints file is empty. Check waypoints_file port in xml.");
      return;
    }

    // Load waypoints from the specified CSV file. If it fails, the node is invalid.
    if (!loadWaypoints()) {
        return;
    }

    // Initialize the action client using the resolved action name.
    ac_ = std::make_unique<actionlib::SimpleActionClient<ugv_bt_interfaces::WaypointAction>>(
      action_name_, true);
    ROS_INFO("Waiting for action server [%s]...", action_name_.c_str());

    // Increase the timeout to give the server more time to start up.
    if (!ac_->waitForServer(ros::Duration(10.0))) {
        ROS_INFO("%sCould not connect to action server [%s]. Node will not be operational.", name_.c_str(), action_name_.c_str());
        ROS_INFO("%sPlease make sure the action server is running and the name is correct.", name_.c_str());
        valid_ = false;
        return;
    }

    ROS_INFO("Action server [%s] connected.", action_name_.c_str());
    valid_ = true;
  }

  /**
   * @brief Defines the input ports of the node.
   * @return A list of ports.
   */
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("action_name", "Name of the ROS action server"),
      BT::InputPort<std::string>("waypoints_file", "Path to the waypoints CSV file relative to the package"),
    };
  }

  /**
   * @brief The main execution function of the node, called once per activation.
   * @return The status of the node (RUNNING, SUCCESS, or FAILURE).
   */
  BT::NodeStatus tick() override {
    setStatus(BT::NodeStatus::RUNNING);

    ros::spinOnce();

    if (!valid_) {
        ROS_INFO("%sNode is not valid. Tick returning FAILURE.", name_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Reset state flags before sending a new goal
    goal_complete_.store(false);
    action_succeeded_.store(false);

    // Build the goal message from the loaded waypoints
    ugv_bt_interfaces::WaypointGoal goal;
    for (const auto& wp : waypoints_) {
      ugv_bt_interfaces::Waypoint msg;
      msg.x   = std::get<0>(wp);
      msg.y   = std::get<1>(wp);
      msg.yaw = std::get<2>(wp);
      goal.waypoints.push_back(msg);
    }

    ROS_INFO("Sending action goal to %s", action_name_.c_str());
    
    // Send the goal to the action server. The doneCb callback will be invoked upon completion
    ac_->sendGoal(
      goal, std::bind(&ActionWaypointClient::doneCb, this, std::placeholders::_1, std::placeholders::_2),
      actionlib::SimpleActionClient<ugv_bt_interfaces::WaypointAction>::SimpleActiveCallback(),
      actionlib::SimpleActionClient<ugv_bt_interfaces::WaypointAction>::SimpleFeedbackCallback());
    
    // Wait until the action is complete
    while (!goal_complete_.load() && ros::ok()) {
      setStatusRunningAndYield();
    }
    
    // Return SUCCESS or FAILURE based on the result from the done callback.
    return action_succeeded_.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  /**
   * @brief Called by the Behavior Tree engine to stop the execution of the node.
   * This is mandatory for asynchronous nodes.
   */
  void halt() override {
    // Only cancel the goal if the node was actually running.
    if (status() == BT::NodeStatus::RUNNING) {
        ROS_INFO("Node halted. Cancelling goal on action server [%s].", 
                 action_name_.c_str());
        ac_->cancelGoal();
    }

    // Halt the coroutine
    CoroActionNode::halt();
  }

private:
  /**
   * @brief Loads waypoints from a CSV file.
   * @return True if loading was successful, false otherwise.
   */
  bool loadWaypoints()
  {
    // Find the package path for ugv_bt_manager
    std::string package_path = ros::package::getPath("ugv_bt_manager");
    if (package_path.empty()) {
        ROS_INFO("Could not find ROS package [ugv_bt_manager].");
        return false;
    }

    std::string full_path = package_path + "/" + waypoints_file_;
    std::ifstream file(full_path);
    if (!file.is_open()) {
        ROS_INFO("Failed to open waypoints file: %s", full_path.c_str());
        return false;
    }

    // Read the file line by line and parse the CSV data
    std::string line;
    int line_num = 0;
    while (std::getline(file, line)) {
      line_num++;
      std::stringstream ss(line);
      std::string x_str, y_str, yaw_str;
      
      // Skip empty lines or lines that start with a comment character '#'
      if (line.empty() || line.find_first_not_of(" \t") == std::string::npos || line[0] == '#') {
          continue;
      }

      if (std::getline(ss, x_str, ',') &&
          std::getline(ss, y_str, ',') &&
          std::getline(ss, yaw_str, ',')) {
        try {
            // Convert string values to double and store them
            waypoints_.emplace_back(std::stod(x_str), std::stod(y_str), std::stod(yaw_str));
        }
        catch (const std::invalid_argument& ia) {
            ROS_INFO("Invalid number in waypoints file at line %d:%s", line_num, line.c_str());
        }
        catch (const std::out_of_range& oor) {
            ROS_INFO("Number out of range in waypoints file at line %d:%s", line_num, line.c_str());
        }
      }
    }

    if (waypoints_.empty()) {
      ROS_INFO("No valid waypoints were loaded from %s", full_path.c_str());
      return false;
    }
    
    ROS_INFO("%ld waypoints loaded from %s", waypoints_.size(), full_path.c_str());
    return true;
  }

  /**
   * @brief Callback executed when the ROS action finishes.
   * @param state The final state of the action.
   * @param result A pointer to the result message.
   */
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const ugv_bt_interfaces::WaypointResultConstPtr& result) {
    ROS_INFO("Action finished with state: %s", state.toString().c_str());

    // Set the result flag based on the action's final state
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Action Succeeded.");
      action_succeeded_.store(true);
    }
    else {
      ROS_INFO("Action Failed/Cancelled/Preempted: %s", state.toString().c_str());
      action_succeeded_.store(false);
    }
    
    // Signal that the action is complete to unblock the tick() method.
    goal_complete_.store(true);
  }

  // Member variables
  std::string name_;
  std::string action_name_, waypoints_file_;
  bool valid_;
  std::unique_ptr<actionlib::SimpleActionClient<ugv_bt_interfaces::WaypointAction>> ac_;
  std::vector<std::tuple<double, double, double>> waypoints_;
  std::atomic_bool goal_complete_;
  std::atomic_bool action_succeeded_;
};

}  // namespace BehaviorTreeNodes
