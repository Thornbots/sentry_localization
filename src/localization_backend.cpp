// Copyright 2026 Thornbots
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

#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_amcl/amcl_node.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_map_server/map_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory_template.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"

// Static composition keeps lifecycle services local and libraries loaded until exit.
// Each node has its own executor; see README.md.
class Backend
{
public:
  ~Backend()
  {
    for (const auto & executor : executors_) {
      executor->cancel();
    }
    for (auto & thread : threads_) {
      thread.join();
    }
    executors_.clear();
    while (!nodes_.empty()) {
      nodes_.pop_back();
    }
  }

  template<typename NodeT>
  void add(const std::string & name)
  {
    // Local name remaps override the process name; YAML retains its node-name scopes.
    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=" + name});
    rclcpp_components::NodeFactoryTemplate<NodeT> factory;
    auto node = factory.create_node_instance(options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node.get_node_base_interface());
    nodes_.push_back(std::move(node));
    executors_.push_back(executor);
  }

  void spin()
  {
    for (size_t i = 0; i + 1 < executors_.size(); ++i) {
      threads_.emplace_back([executor = executors_[i]] {executor->spin();});
    }
    executors_.back()->spin();
  }

private:
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes_;
  std::vector<std::shared_ptr<rclcpp::Executor>> executors_;
  std::vector<std::thread> threads_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  Backend backend;
  int result = 0;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 2 || (args[1] != "amcl" && args[1] != "slam")) {
      throw std::invalid_argument("Expected backend argument: amcl or slam");
    }
    if (args[1] == "amcl") {
      backend.add<nav2_map_server::MapServer>("map_server");
      backend.add<nav2_amcl::AmclNode>("amcl");
    } else {
      backend.add<slam_toolbox::AsynchronousSlamToolbox>("slam_toolbox");
    }
    backend.add<nav2_lifecycle_manager::LifecycleManager>(
      args[1] == "amcl" ? "lifecycle_manager_localization" : "lifecycle_manager_slam");
    backend.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_backend"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
