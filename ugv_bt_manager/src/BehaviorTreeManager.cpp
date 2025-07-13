// Copyright 2022 Fraunhofer FKIE
// Copyright 2025 Matheus França (modified for UGV Decision Manager)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "BehaviorTreeManager.h"
#include <fstream>
#include <sstream>


// Helper function to read a file's content into a string.
std::string readContentFile(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    ROS_ERROR("Could not open file: %s", path.c_str());
    return "";
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}


BehaviorTreeManager::BehaviorTreeManager() {
  ros::NodeHandle nh("~");

  pub_bt_status = nh.advertise<ugv_bt_interfaces::BTStatus>("bt_status", 5, false);

  // generic subscriber for starting the tree
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback_start;
  callback_start = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { callbackStart(msg); };
  sub_start = nh.subscribe("start_tree", 1, callback_start);

  // generic subscriber for stopping the tree
  boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback_stop;
  callback_stop = [&](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { callbackStop(msg); };
  sub_stop = nh.subscribe("stop_tree", 1, callback_stop);

  // Removed: s_bt_publisher logic, as the header is gone.
};

BehaviorTreeManager::~BehaviorTreeManager() {
  sub_start.shutdown();
  sub_stop.shutdown();
};

void BehaviorTreeManager::callbackStart(const topic_tools::ShapeShifter::ConstPtr& msg) {
  ROS_INFO("New request to start the tree execution");

  if (is_tree_enabled) {
    ROS_INFO("Tree is enabled, ignoring request");
    return;
  }

  {
    std::lock_guard<std::mutex> lck(mutex_tree);

    // reinitialize the tree and loggers
    reinitializeTree();
    initializeLoggers();
    ROS_WARN("[%s] has been started!", ros::this_node::getName().c_str());
  }

  reportBTState();

  is_tree_enabled = true;
}

void BehaviorTreeManager::callbackStop(const topic_tools::ShapeShifter::ConstPtr& msg) {
  ROS_INFO("New request to stop the tree execution");

  if (!is_tree_enabled)
  {
    ROS_INFO("Tree is disabled, ignoring request");
    return;
  }

  // stop BT nodes
  tree->haltTree();

  // disable loggers
  if (std_cout_logger)
    std_cout_logger->setEnabled(false);
  if (file_logger)
    file_logger->setEnabled(false);
  if (minitrace_logger)
    minitrace_logger->setEnabled(false);
  if (zmq_logger)
    zmq_logger->setEnabled(false);
  
  // Removed ros_msg_logger logic

  // wait for the nodes to terminate (useful for async nodes)
  ros::Duration(params.seconds_wait_after_stop).sleep();

  must_delete_tree = true;
  is_tree_enabled = false;

  reportBTState();
}

void BehaviorTreeManager::reportBTState()
{
  ugv_bt_interfaces::BTStatus bt_status_msg;
  bt_status_msg.tree_file = params.tree_static_file;

  // read file (only once) and send XML content
  if (file_tree_content.empty() && !params.tree_static_file.empty())
    file_tree_content = readContentFile(params.tree_static_file);

  bt_status_msg.tree_xml = file_tree_content;

  bt_status_msg.header.stamp = ros::Time::now();
  bt_status_msg.tree_is_running = is_tree_enabled;
  pub_bt_status.publish(bt_status_msg);
}

void BehaviorTreeManager::spin() {
  // check if tree must start automatically
  is_tree_enabled = params.start_tree_at_init;

  ros::Rate r(params.rate);

  while (ros::ok()) {
    if (tree) {
      if (is_tree_enabled) {
        ROS_DEBUG("Starting execution of [root] node on tree");
        
        // Capture the status of the tree tick
        const BT::NodeStatus tree_status = tree->tickRoot();

        // Check the final status of the tree
        switch (tree_status) {
          case BT::NodeStatus::SUCCESS:
            ROS_INFO("Behavior Tree returned SUCCESS. Stopping execution.");
            is_tree_enabled = false;
            tree->haltTree();
            must_delete_tree = true; // To trigger cleanup
            break;

          case BT::NodeStatus::FAILURE:
            // The tree is still running, do nothing and wait for the next tick.
            break;

          case BT::NodeStatus::RUNNING:
            // The tree is still running, do nothing and wait for the next tick.
            break;

          default:
            // Should not happen
            ROS_ERROR("Behavior Tree returned an unexpected status. Stopping.");
            is_tree_enabled = false;
            tree->haltTree();
            must_delete_tree = true; // To trigger cleanup
            break;
        }
      }
      else {
        ROS_DEBUG("Tree execution is disabled");

        // recreate the tree and loggers
        if (must_delete_tree) {
          resetLoggers();
          {
            std::lock_guard<std::mutex> lck(mutex_tree);
            tree.reset(nullptr);
          }

          ROS_WARN("[%s] has been stopped!", ros::this_node::getName().c_str());
          must_delete_tree = false;
        }
      }
    }

    reportBTState();
    ros::spinOnce();
    r.sleep();
  }
}

void BehaviorTreeManager::registerActionPlugins() {
  // Register common conditions
  bt_factory.registerFromPlugin("libBehaviorTreeNodes_dyn.so");

  for (const std::string& ap : params.action_plugins)
  {
    ROS_DEBUG("Loading plugin: %s", ap.c_str());
    bt_factory.registerFromPlugin(ap);
  }
}

void BehaviorTreeManager::initializeTreeFromText(const std::string static_tree)
{
  ROS_INFO("Initializing tree from text");
  tree.reset(new BT::Tree(bt_factory.createTreeFromText(static_tree)));
  current_static_tree = static_tree;
  current_path_file.clear();
}

void BehaviorTreeManager::initializeTreeFromFile(const std::string path_file)
{
  ROS_INFO("Initializing tree from file: %s", path_file.c_str());
  tree.reset(new BT::Tree(bt_factory.createTreeFromFile(path_file)));
  current_path_file = path_file;
  current_static_tree.clear();
}

bool BehaviorTreeManager::reinitializeTree()
{
  // create a new tree instance
  if (!tree)
  {
    tree.reset(new BT::Tree());
  }

  if (!current_static_tree.empty())
  {
    initializeTreeFromText(current_static_tree);
  }
  else if (!current_path_file.empty())
  {
    initializeTreeFromFile(current_path_file);
  }
  return false;
}

void BehaviorTreeManager::initializeLoggers()
{
  if (!tree)
  {
    ROS_ERROR("Tree is not initialized, cannot create loggers.");
    return;
  }

  if (params.StdCoutLogger_enabled)
  {
    ROS_INFO("Creating StdCoutLogger");
    std_cout_logger.reset(new BT::StdCoutLogger(*tree));
  }

  if (params.FileLogger_enabled)
  {
    ROS_INFO("Creating FileLogger on file: %s", params.FileLogger_file_path.c_str());
    file_logger.reset(new BT::FileLogger(*tree, params.FileLogger_file_path.c_str()));
  }

  if (params.MinitraceLogger_enabled)
  {
    ROS_INFO("Creating MinitraceLogger on file: %s", params.MinitraceLogger_file_path.c_str());
    minitrace_logger.reset(new BT::MinitraceLogger(*tree, params.MinitraceLogger_file_path.c_str()));
  }

  if (params.PublisherZMQ_enabled)
  {
    ROS_INFO("Creating PublisherZMQ in ports: [publisher: %d server: %d]",
             params.PublisherZMQ_publisher_port, params.PublisherZMQ_server_port);
    zmq_logger.reset(new BT::PublisherZMQ(*tree, params.PublisherZMQ_max_msg_per_second,
                                          params.PublisherZMQ_publisher_port, params.PublisherZMQ_server_port));
  }
}

void BehaviorTreeManager::resetLoggers()
{
  if (std_cout_logger) {
    std_cout_logger->setEnabled(false);
    std_cout_logger.reset(nullptr);
  }

  if (file_logger) {
    file_logger->setEnabled(false);
    file_logger.reset(nullptr);
  }

  if (minitrace_logger) {
    minitrace_logger->setEnabled(false);
    minitrace_logger.reset(nullptr);
  }

  if (zmq_logger) {
    zmq_logger->setEnabled(false);
    zmq_logger.reset(nullptr);
  }
}
