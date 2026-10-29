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
#include <fcntl.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/fs.h>
#include <sys/syscall.h>
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_localization/map_image.hpp"
#include "sentry_localization/srv/save_map.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"

using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;
using SaveMap = sentry_localization::srv::SaveMap;
using Clock = std::chrono::steady_clock;

class MapAutosaver : public rclcpp::Node
{
public:
  MapAutosaver()
  : Node("map_autosaver")
  {
    declare_parameter<std::string>("save_dir", "/workspaces/isaac_ros-dev/maps");
    declare_parameter<double>("period_s", 30.0);
    declare_parameter<bool>("periodic_enabled", true);
    timeout_s_ = declare_parameter<double>("serialize_timeout_s", 60.0);
    max_age_s_ = declare_parameter<double>("max_map_age_s", 30.0);
    if (!std::isfinite(timeout_s_) || timeout_s_ <= 0 ||
      !std::isfinite(max_age_s_) || max_age_s_ <= 0)
    {
      throw std::invalid_argument("save timeouts must be finite and positive");
    }
    started_stamp_ = now().nanoseconds();
    save_service_ = create_service<SaveMap>(
      "/map_autosaver/save_now",
      [this](std::shared_ptr<rclcpp::Service<SaveMap>>,
      std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<SaveMap::Request> request) {
        save_now(header, request);
      });
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (explicit_ && !explicit_->responded && since(explicit_->started) >= timeout_s_) {
          respond(false, "serialization timed out; saver busy until backend finishes");
        }
      });
    if (!get_parameter("periodic_enabled").as_bool()) {
      serialize_ = create_client<SerializePoseGraph>("/slam_toolbox/serialize_map");
      subscribe_map();
      return;
    }
    char stamp[32];
    const std::time_t now = std::time(nullptr);
    std::tm tm_now{};
    localtime_r(&now, &tm_now);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_now);
    const auto run_dir = std::filesystem::path(get_parameter("save_dir").as_string()) / stamp;
    std::filesystem::create_directories(run_dir);
    base_ = (run_dir / "map").string();
    serialize_ = create_client<SerializePoseGraph>("/slam_toolbox/serialize_map");
    subscribe_map();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(get_parameter("period_s").as_double()),
      [this]() {
        if (!explicit_ && !serialize_pending_) {
          save_image();
          serialize_graph();
        }
      });
    RCLCPP_INFO(get_logger(), "saving the map to %s.*", base_.c_str());
  }

private:
  struct ExplicitSave
  {
    std::filesystem::path target, stage;
    std::shared_ptr<rmw_request_id_t> header;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
    Clock::time_point started;
    int64_t minimum_stamp;
    bool validated, responded = false;
  };

  void subscribe_map()
  {
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {map_ = msg;});
  }

  void validate_map(const nav_msgs::msg::OccupancyGrid & map, int64_t minimum) const
  {
    const auto stamp = rclcpp::Time(map.header.stamp).nanoseconds();
    const double age = (get_clock()->now().nanoseconds() - stamp) / 1e9;
    const auto & i = map.info;
    const auto & o = i.origin;
    const auto & q = o.orientation;
    if (map.header.frame_id != "map" || stamp < minimum || age < 0 || age > max_age_s_) {
      throw std::runtime_error("map is stale, predates start/reset, future-dated or outside map frame");
    }
    if (!i.width || !i.height || map.data.size() != static_cast<size_t>(i.width) * i.height ||
      !std::isfinite(i.resolution) || i.resolution <= 0 ||
      !std::isfinite(o.position.x) || !std::isfinite(o.position.y) ||
      !std::isfinite(o.position.z) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      q.x != 0 || q.y != 0 || std::abs(q.z * q.z + q.w * q.w - 1.0) > 1e-3 ||
      std::any_of(map.data.begin(), map.data.end(), [](int v) {return v < -1 || v > 100;}) ||
      std::none_of(map.data.begin(), map.data.end(), [](int v) {return v >= 0;}))
    {
      throw std::runtime_error("invalid or entirely unknown occupancy grid");
    }
  }

  void respond(bool success, const std::string & message)
  {
    SaveMap::Response response;
    response.success = success;
    response.message = message;
    explicit_->responded = true;
    save_service_->send_response(*explicit_->header, response);
  }

  void cleanup_explicit()
  {
    std::error_code error;
    std::filesystem::remove_all(explicit_->stage, error);
    explicit_.reset();
  }

  void finish_explicit(rclcpp::Client<SerializePoseGraph>::SharedFuture future)
  {
    if (explicit_->responded) {
      cleanup_explicit();
      return;
    }
    try {
      const auto result = future.get();
      if (!result || result->result != 0 || since(explicit_->started) >= timeout_s_) {
        throw std::runtime_error("serialize_map failed or exceeded timeout");
      }
      validate_map(*explicit_->map, explicit_->minimum_stamp);
      const auto base = (explicit_->stage / "map").string();
      const auto & i = explicit_->map->info;
      const auto & o = i.origin;
      const auto pgm = sentry_localization::to_pgm(explicit_->map->data, i.width, i.height);
      const auto yaml = sentry_localization::to_yaml(
        "map.pgm", i.resolution, o.position.x, o.position.y, o.orientation.z, o.orientation.w);
      sentry_localization::write_atomic(base + ".pgm", pgm);
      sentry_localization::write_atomic(base + ".yaml", yaml);
      for (const auto & suffix : {".posegraph", ".data", ".pgm", ".yaml"}) {
        const std::filesystem::path file(base + suffix);
        if (!std::filesystem::is_regular_file(file) || !std::filesystem::file_size(file)) {
          throw std::runtime_error("missing or empty serialized map file: " + file.string());
        }
      }
      for (const auto & item : {std::make_pair(".pgm", pgm), std::make_pair(".yaml", yaml)}) {
        std::ifstream file(base + item.first, std::ios::binary);
        const std::string actual((std::istreambuf_iterator<char>(file)), {});
        if (actual != item.second) {
          throw std::runtime_error("occupancy output verification failed");
        }
      }
      sentry_localization::write_atomic(
        (explicit_->stage / "save.json").string(),
        std::string("{\"validated\":") + (explicit_->validated ? "true" : "false") + "}\n");
      // Refuse even an empty destination created after request acceptance.
#ifdef __linux__
      const int renamed = static_cast<int>(::syscall(
          SYS_renameat2, AT_FDCWD, explicit_->stage.c_str(),
          AT_FDCWD, explicit_->target.c_str(), RENAME_NOREPLACE));
#else
      const int renamed = ::renamex_np(
        explicit_->stage.c_str(), explicit_->target.c_str(), RENAME_EXCL);
#endif
      if (renamed != 0) {
        throw std::runtime_error("cannot publish snapshot: destination exists or rename failed");
      }
      respond(true, "saved " + explicit_->target.string());
    } catch (const std::exception & error) {
      respond(false, error.what());
    }
    cleanup_explicit();
  }

  void save_now(
    std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<SaveMap::Request> & request)
  {
    if (explicit_ || serialize_pending_) {
      SaveMap::Response response;
      response.success = false;
      response.message = "map saver busy";
      save_service_->send_response(*header, response);
      return;
    }
    explicit_ = std::make_unique<ExplicitSave>();
    explicit_->header = header;
    explicit_->started = Clock::now();
    try {
      const auto minimum = rclcpp::Time(request->minimum_stamp).nanoseconds();
      if (minimum <= 0) {
        throw std::runtime_error("minimum_stamp must identify session start/latest SLAM reset");
      }
      explicit_->minimum_stamp = std::max(minimum, started_stamp_);
      if (!map_ || !serialize_->service_is_ready()) {
        throw std::runtime_error("map or serialize_map service unavailable");
      }
      validate_map(*map_, explicit_->minimum_stamp);
      explicit_->map = map_;
      explicit_->validated = request->validated;
      if (request->directory.empty()) {
        throw std::runtime_error("directory cannot be empty");
      }
      explicit_->target = std::filesystem::absolute(request->directory).lexically_normal();
      if (std::filesystem::symlink_status(explicit_->target).type() !=
        std::filesystem::file_type::not_found)
      {
        throw std::runtime_error("destination already exists");
      }
      std::filesystem::create_directories(explicit_->target.parent_path());
      auto pattern = (explicit_->target.parent_path() / ".mapping-XXXXXX").string();
      if (!::mkdtemp(pattern.data())) {
        throw std::runtime_error("cannot create private staging directory");
      }
      explicit_->stage = pattern;
      auto serialize = std::make_shared<SerializePoseGraph::Request>();
      serialize->filename = (explicit_->stage / "map").string();
      serialize_->async_send_request(serialize,
        [this](rclcpp::Client<SerializePoseGraph>::SharedFuture f) {finish_explicit(f);});
    } catch (const std::exception & error) {
      respond(false, error.what());
      cleanup_explicit();
    }
  }

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

  std::unique_ptr<ExplicitSave> explicit_;
  rclcpp::Service<SaveMap>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  int64_t started_stamp_;
  double timeout_s_, max_age_s_;
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
