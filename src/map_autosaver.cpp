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

// Save slam_toolbox's map every period_s into <save_dir>/<boot time>/.
//
// map.posegraph/.data via serialize_map (loadable with load_map:=true), and
// map.pgm/.yaml written here from /map. Not slam_toolbox's save_map: it
// starts a map_saver_cli process that must find /map within 2 s, and on the
// sentry often didn't. Each save overwrites the last, so a power cut loses
// at most period_s. A serialize still running skips its next turn.
#include <chrono>
#include <cstdio>
#include <ctime>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_localization/map_image.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;
using Clock = std::chrono::steady_clock;

class MapAutosaver : public rclcpp::Node
{
public:
  MapAutosaver()
  : Node("map_autosaver")
  {
    declare_parameter<std::string>("save_dir", "/workspaces/isaac_ros-dev/maps");
    declare_parameter<double>("period_s", 30.0);
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::tm tm_now{};
    localtime_r(&now, &tm_now);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_now);
    const auto run_dir = std::filesystem::path(get_parameter("save_dir").as_string()) / stamp;
    std::filesystem::create_directories(run_dir);
    base_ = (run_dir / "map").string();
    serialize_ = create_client<SerializePoseGraph>("/slam_toolbox/serialize_map");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = msg;});
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(get_parameter("period_s").as_double()),
      [this]() {
        save_image();
        serialize_graph();
      });
    RCLCPP_INFO(get_logger(), "saving the map to %s.*", base_.c_str());
  }

private:
  static double since(Clock::time_point t)
  {
    return std::chrono::duration<double>(Clock::now() - t).count();
  }

  void save_image()
  {
    const auto m = map_;
    if (!m || m == map_saved_) {
      return;
    }
    const auto started = Clock::now();
    const auto & info = m->info;
    const auto & origin = info.origin;
    try {
      sentry_localization::write_atomic(
        base_ + ".pgm", sentry_localization::to_pgm(m->data, info.width, info.height));
      sentry_localization::write_atomic(
        base_ + ".yaml",
        sentry_localization::to_yaml(
          "map.pgm", info.resolution, origin.position.x, origin.position.y,
          origin.orientation.z, origin.orientation.w));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "map.pgm failed: %s", e.what());
      return;
    }
    map_saved_ = m;
    RCLCPP_INFO(
      get_logger(), "map.pgm ok (%ux%u) in %.2f s", info.width, info.height, since(started));
  }

  void serialize_graph()
  {
    if (serialize_pending_ || !serialize_->service_is_ready()) {
      return;
    }
    const auto started = Clock::now();
    auto req = std::make_shared<SerializePoseGraph::Request>();
    req->filename = base_;
    serialize_pending_ = true;
    serialize_->async_send_request(
      req, [this, started](rclcpp::Client<SerializePoseGraph>::SharedFuture f) {
        serialize_pending_ = false;
        const double took = since(started);
        const auto resp = f.get();
        if (resp && resp->result == 0) {
          RCLCPP_INFO(get_logger(), "serialize_map ok in %.1f s", took);
        } else if (resp) {
          RCLCPP_WARN(
            get_logger(), "serialize_map failed (%d) after %.1f s", resp->result, took);
        } else {
          RCLCPP_WARN(get_logger(), "serialize_map failed (None) after %.1f s", took);
        }
      });
  }

  std::string base_;
  rclcpp::Client<SerializePoseGraph>::SharedPtr serialize_;
  bool serialize_pending_ = false;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_saved_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAutosaver>());
  rclcpp::shutdown();
  return 0;
}
