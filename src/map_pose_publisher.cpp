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

// One map pose result per local odometry tick. Unavailable global data carries
// NaN position and infinite covariance; consumers must reject it. See README.md.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <deque>
#include <limits>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/header.hpp"
#include "sentry_localization/map_pose_core.hpp"
#include "std_srvs/srv/empty.hpp"

using sentry_localization::Cov2;
using sentry_localization::Pose2D;

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class MapPosePublisher : public rclcpp::Node
{
public:
  MapPosePublisher()
  : Node("map_pose_publisher")
  {
    declare_parameter<std::string>("input_topic", "/localization/odom");
    declare_parameter<std::string>("output_topic", "/localization/map_odom");
    declare_parameter<std::string>("backend_pose_topic", "/amcl_pose");
    declare_parameter<std::string>("map_frame", "map");
    max_backend_age_s_ = declare_parameter("max_backend_age_s", 1.0);
    const auto refresh_service = declare_parameter("nomotion_update_service", std::string(""));
    if (!refresh_service.empty()) {
      refresh_client_ = create_client<std_srvs::srv::Empty>(refresh_service);
    }
    map_frame_ = get_parameter("map_frame").as_string();
    pub_ = create_publisher<nav_msgs::msg::Odometry>(get_parameter("output_topic").as_string(), 10);
    // Lockstep simulation only (backend_result_topic): the backend answers every scan
    // (empty frame_id: no pose). A tick waits until every scan up to backend_delay_s
    // before it is answered and uses only those, so its compute time picks no tick.
    const auto result_topic = declare_parameter("backend_result_topic", std::string(""));
    backend_delay_s_ = declare_parameter("backend_delay_s", 0.0);
    const auto keep = [this](const geometry_msgs::msg::PoseWithCovarianceStamped & msg) {
        if (msg.header.frame_id != map_frame_) {return;}
        const auto & c = msg.pose.covariance;
        const double t = rclcpp::Time(msg.header.stamp).seconds();
        if (!backend_.empty() && t < backend_.back().t) {backend_.clear();}
        const auto & p = msg.pose.pose;
        backend_.push_back({t, Cov2{{{c[0], c[1]}, {c[6], c[7]}}}, c[35],
          {p.position.x, p.position.y, yaw_of(p.orientation)}});
        while (backend_.size() > 100) {backend_.pop_front();}
      };
    if (result_topic.empty()) {
      backend_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        get_parameter("backend_pose_topic").as_string(), 10,
        [keep](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {keep(*msg);});
    } else {
      // The backend refreshes itself on scan stamps (refresh_after_s), not on a service call.
      refresh_client_.reset();
      backend_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        result_topic, rclcpp::QoS(100).reliable(),
        [this, keep](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
          keep(*msg);
          const double t = rclcpp::Time(msg->header.stamp).seconds();
          answered_ = std::max(answered_.value_or(t), t);
          ++answers_;
          drain();
        });
      // A lockstep chain acks this, so each scan is seen here before later ticks run.
      scan_seen_pub_ = create_publisher<std_msgs::msg::Header>(
        declare_parameter("scan_seen_topic", std::string("~/scan_seen")), 100);
      // Reliable, like the backend's scan subscription: a scan seen here must reach it.
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        declare_parameter("scan_topic", std::string("/scan")), rclcpp::QoS(100).reliable().transient_local(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
          seen_.push_back(rclcpp::Time(scan->header.stamp).seconds());
          while (seen_.size() > 1000) {seen_.pop_front();}
          scan_seen_pub_->publish(scan->header);
        });
      // Only notices a backend that has gone away; data events drain otherwise.
      drain_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {drain();});
      stats_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::seconds(5), [this]() {
          RCLCPP_INFO(get_logger(), "backend results: %zu ticks waited for one; %zu answers",
            deferred_, answers_);
        });
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("input_topic").as_string(), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        pending_.push_back(msg);
        drain();
        deferred_ += !pending_.empty() && pending_.back() == msg;
      });
  }

private:
  // Newest backend stamp tick t may use; the tolerance keeps whole-ms stamps exact.
  double due(double t) const
  {
    return backend_delay_s_ > 0.0 ? t - backend_delay_s_ + 1e-9 : t;
  }

  // Lockstep: every scan the backend should have answered by tick t has been answered.
  bool backend_ready(double t) const
  {
    if (!scan_sub_ || !answered_ || backend_sub_->get_publisher_count() == 0) {
      return true;
    }
    for (auto i = seen_.rbegin(); i != seen_.rend(); ++i) {
      if (*i <= due(t)) {return *i <= *answered_;}
    }
    return true;
  }

  void drain()
  {
    while (!pending_.empty() &&
      backend_ready(rclcpp::Time(pending_.front()->header.stamp).seconds()))
    {
      const auto msg = pending_.front();
      pending_.pop_front();
      odom_cb(*msg);
    }
  }

  std::optional<Pose2D> local_at(double t) const
  {
    if (local_.empty() || t < local_.front().t || t > local_.back().t) {return std::nullopt;}
    const auto b = std::lower_bound(local_.begin(), local_.end(), t,
      [](const Local & a, double t) {return a.t < t;});
    if (b == local_.begin() || b->t == t) {return b->pose;}
    const auto & a = *(b-1);
    const double f = (t-a.t)/(b->t-a.t);
    const double dyaw = b->pose.yaw-a.pose.yaw;
    return Pose2D{a.pose.x + f*(b->pose.x-a.pose.x), a.pose.y + f*(b->pose.y-a.pose.y),
      a.pose.yaw + f*std::atan2(std::sin(dyaw), std::cos(dyaw))};
  }

  void odom_cb(const nav_msgs::msg::Odometry & msg)
  {
    const double t = rclcpp::Time(msg.header.stamp).seconds();
    if (last_tick_ && t < *last_tick_) {backend_.clear(); local_.clear(); last_refresh_.reset();}
    if (local_frame_ != msg.header.frame_id) {
      local_.clear(); backend_.clear(); local_frame_ = msg.header.frame_id;
    }
    const auto & input = msg.pose.pose;
    if (!local_.empty() && local_.back().t == t) {local_.pop_back();}
    const bool valid_input = std::isfinite(input.position.x) && std::isfinite(input.position.y) &&
      std::isfinite(yaw_of(input.orientation));
    if (valid_input) {
      local_.push_back({t, {input.position.x, input.position.y, yaw_of(input.orientation)}});
    }
    while (!local_.empty() && local_.front().t < t - 2.0) {local_.pop_front();}
    last_tick_ = t;
    const Backend * backend = nullptr;
    for (auto i = backend_.rbegin(); i != backend_.rend(); ++i) {
      if (i->t <= due(t)) {backend = &*i; break;}
    }
    // AMCL otherwise stops pose/covariance updates while parked. Ask for a
    // new scan update rather than pretending old covariance is still fresh.
    if (refresh_client_ && (!backend || t-backend->t >= 0.5) &&
      (!last_refresh_ || t-*last_refresh_ >= 0.5) && refresh_client_->service_is_ready())
    {
      if (refresh_id_) {refresh_client_->remove_pending_request(*refresh_id_);}
      refresh_id_ = refresh_client_->async_send_request(
        std::make_shared<std_srvs::srv::Empty::Request>()).request_id;
      last_refresh_ = t;
    }
    const auto at_backend = backend ? local_at(backend->t) : std::nullopt;
    const auto mo = backend && at_backend ? std::optional<Pose2D>(
      sentry_localization::map_correction(backend->pose, *at_backend)) : std::nullopt;
    if (!valid_input || !mo || !backend || !sentry_localization::usable_backend_covariance(
      backend->cov, backend->yaw_var, backend->t, t, max_backend_age_s_)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Global pose unavailable: backend=%d paired_local=%d backend_age=%.3f covariance_valid=%d",
        backend != nullptr, at_backend.has_value(), backend ? t-backend->t : -1.0,
        backend && sentry_localization::usable_backend_covariance(
          backend->cov, backend->yaw_var, backend->t, t, max_backend_age_s_));
      nav_msgs::msg::Odometry invalid;
      invalid.header = msg.header;
      invalid.header.frame_id = map_frame_;
      invalid.child_frame_id = msg.child_frame_id;
      invalid.pose.pose.position.x = invalid.pose.pose.position.y =
        std::numeric_limits<double>::quiet_NaN();
      invalid.pose.covariance[0] = invalid.pose.covariance[7] =
        invalid.pose.covariance[35] = std::numeric_limits<double>::infinity();
      pub_->publish(invalid);
      return;
    }
    const auto & p = msg.pose.pose;
    const Pose2D pose = sentry_localization::compose(*mo, {p.position.x, p.position.y, yaw_of(p.orientation)});
    const auto & c = msg.pose.covariance;
    const Cov2 rot = sentry_localization::rotate_cov_xy(Cov2{{{c[0], c[1]}, {c[6], c[7]}}}, mo->yaw);
    const Cov2 & b = backend->cov;

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = msg.child_frame_id;
    out.pose.pose.position.x = pose.x;
    out.pose.pose.position.y = pose.y;
    out.pose.pose.position.z = p.position.z;
    out.pose.pose.orientation.z = std::sin(pose.yaw / 2.0);
    out.pose.pose.orientation.w = std::cos(pose.yaw / 2.0);
    out.pose.covariance = c;
    out.pose.covariance[0] = rot[0][0] + b[0][0];
    out.pose.covariance[1] = rot[0][1] + b[0][1];
    out.pose.covariance[6] = rot[0][1] + b[0][1];
    out.pose.covariance[7] = rot[1][1] + b[1][1];
    out.pose.covariance[35] = std::max(0.0, c[35] + backend->yaw_var);
    out.twist = msg.twist;
    pub_->publish(out);
  }

  std::string map_frame_;
  struct Backend {double t; Cov2 cov; double yaw_var; Pose2D pose;};
  struct Local {double t; Pose2D pose;};
  std::deque<Local> local_;
  std::string local_frame_;
  std::deque<Backend> backend_;
  std::optional<double> last_tick_;
  double max_backend_age_s_;
  double backend_delay_s_{0.0};
  std::optional<double> answered_;
  std::deque<double> seen_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> pending_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr scan_seen_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr drain_timer_, stats_timer_;
  size_t deferred_{0}, answers_{0};
  std::optional<double> last_refresh_;
  std::optional<int64_t> refresh_id_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr refresh_client_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr backend_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
