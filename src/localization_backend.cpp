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

#include <chrono>
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
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

// nav2_amcl caches each scan frame's base->laser x/y at its first scan. Our
// lidar rides the head, about 0.22 m off root's yaw axis, so refresh that
// mount from TF at every scan stamp before AMCL's own callback. AMCL already
// re-reads the mount yaw per scan. See README.md (config/amcl.yaml).
class TrackedMountAmcl : public nav2_amcl::AmclNode
{
public:
  using nav2_amcl::AmclNode::AmclNode;

protected:
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = nav2_amcl::AmclNode::on_activate(state);
    laser_scan_connection_.disconnect();
    laser_scan_connection_ = laser_scan_filter_->registerCallback(
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {receive(scan);});
    return result;
  }

private:
  void receive(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const auto found = frame_to_laser_.find(scan->header.frame_id);
    if (found != frame_to_laser_.end()) {
      std::string frame = scan->header.frame_id;
      if (!frame.empty() && frame.front() == '/') {
        frame.erase(0, 1);
      }
      geometry_msgs::msg::TransformStamped mount;
      try {
        mount = tf_buffer_->lookupTransform(
          base_frame_id_, frame, tf2_ros::fromMsg(scan->header.stamp));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Skipping scan without stamped %s->%s: %s",
          base_frame_id_.c_str(), frame.c_str(), error.what());
        return;
      }
      pf_vector_t pose = pf_vector_zero();
      pose.v[0] = mount.transform.translation.x;
      pose.v[1] = mount.transform.translation.y;
      lasers_[found->second]->SetLaserPose(pose);
    }
    laserReceived(scan);
  }
};

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

  bool spin(bool switchable)
  {
    auto control = std::make_shared<rclcpp::Node>("localization_switch");
    bool requested = false;
    auto service = control->create_service<std_srvs::srv::Trigger>(
      "/localization/start_mapping",
      [&requested, switchable](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        requested = switchable;
        response->success = true;
        response->message = switchable ? "Localization handoff requested" : "Already mapping";
      });
    for (size_t i = 0; i < executors_.size(); ++i) {
      threads_.emplace_back([executor = executors_[i]] {executor->spin();});
    }
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(control);
    while (rclcpp::ok() && !requested) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    if (requested) {rclcpp::shutdown();}
    return requested;
  }

private:
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes_;
  std::vector<std::shared_ptr<rclcpp::Executor>> executors_;
  std::vector<std::thread> threads_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    Backend backend;
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 2 || (args[1] != "amcl" && args[1] != "slam" &&
      args[1] != "mapping" && args[1] != "none"))
    {
      throw std::invalid_argument("Expected backend argument: amcl, slam, mapping or none");
    }
    if (args[1] == "amcl") {
      backend.add<nav2_map_server::MapServer>("map_server");
      backend.add<TrackedMountAmcl>("amcl");
    } else if (args[1] != "none") {
      backend.add<slam_toolbox::AsynchronousSlamToolbox>("slam_toolbox");
    }
    if (args[1] != "none") {
      backend.add<nav2_lifecycle_manager::LifecycleManager>(
        args[1] == "amcl" ? "lifecycle_manager_localization" : "lifecycle_manager_slam");
    }
    // Exit only after all old components are destroyed; launch owns the next backend.
    result = backend.spin(args[1] != "mapping") ? 42 : 0;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_backend"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
