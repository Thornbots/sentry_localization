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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_localization/ros_filter_types.hpp>
#include <std_msgs/msg/header.hpp>

namespace sentry_localization
{
// The installed RosEkf owns all estimation and publication. This adapter only
// orders complete captured inputs before calling its public update API.
class LockstepEkf : public robot_localization::RosEkf
{
public:
  LockstepEkf()
  : robot_localization::RosEkf(rclcpp::NodeOptions().arguments({"ekf_filter_node"})) {}

  void start()
  {
    initialize();
    timer_->cancel();
    topic_subs_.clear();
    if (!get_parameter("use_sim_time").as_bool() || now().nanoseconds() > 5000000) {
      throw std::runtime_error("lockstep EKF must initialize at simulation startup");
    }
    if (frequency_ != 30.0 || use_control_) {
      throw std::invalid_argument("lockstep EKF requires the 30 Hz odometry-only profile");
    }
    for (const auto * extra : {"odom2", "pose0", "twist0", "imu0", "accel0"}) {
      std::string topic;
      if (get_parameter(extra, topic) && !topic.empty()) {
        throw std::invalid_argument("unsupported lockstep EKF input: " + std::string(extra));
      }
    }
    period_ns_ = static_cast<int64_t>(1e9 / frequency_);
    if (period_ns_ < 1000000) {throw std::invalid_argument("EKF frequency exceeds physics rate");}
    input_ack_ = create_publisher<std_msgs::msg::Header>("/sim/lockstep/ekf_input", 100);
    update_ack_ = create_publisher<std_msgs::msg::Header>("/sim/lockstep/ekf", 100);
    for (size_t index = 0; index < 2; ++index) {
      const auto prefix = "odom" + std::to_string(index);
      topics_[index] = get_parameter(prefix).as_string();
      pose_data_.push_back(callback_data(prefix, true));
      twist_data_.push_back(callback_data(prefix, false));
      subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(topics_[index], 100,
        [this, index](nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (!inputs_.emplace(std::make_pair(stamp, index), msg).second) {
            throw std::runtime_error("lockstep timeout: duplicate EKF input");
          }
          if (inputs_.size() > 1000) {throw std::runtime_error("lockstep timeout: EKF input overflow");}
          if (index == 0) {input_ack_->publish(msg->header);}
          scan_receipts();
          drain();
        }));
    }
    subs_.push_back(create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/sim/lockstep/scan_consumed", 100,
        [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
          const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
          if (msg->status.size() != 1 ||
            (msg->status[0].message != "published" && msg->status[0].message != "no_output"))
          {throw std::runtime_error("invalid scan consumption receipt");}
          receipts_[stamp] = msg->status[0].message == "published";
          if (receipts_.size() > 1000) {
            throw std::runtime_error("lockstep timeout: EKF scan receipt overflow");
          }
          scan_receipts();
          drain();
        }));
    subs_.push_back(create_subscription<std_msgs::msg::Header>("/mcb_emulator/consumed_tick", 100,
        [this](std_msgs::msg::Header::ConstSharedPtr msg) {
          watermark_ns_ = std::max(watermark_ns_, rclcpp::Time(msg->stamp).nanoseconds());
          drain();
        }));
    retry_ = create_wall_timer(std::chrono::milliseconds(1), [this] {drain();});
  }

private:
  robot_localization::CallbackData callback_data(const std::string & prefix, bool pose)
  {
    auto update = get_parameter(prefix + "_config").as_bool_array();
    if (update.size() != 15) {throw std::invalid_argument("EKF input config must have 15 entries");}
    for (size_t i = 0; i < update.size(); ++i) {
      if ((pose && i >= 6) || (!pose && (i < 6 || i >= 12))) {update[i] = false;}
    }
    const auto suffix = pose ? "_pose" : "_twist";
    double rejection = std::numeric_limits<double>::max();
    get_parameter(prefix + suffix + "_rejection_threshold", rejection);
    bool child_frame = false;
    get_parameter(prefix + "_pose_use_child_frame", child_frame);
    return robot_localization::CallbackData(
      get_parameter(prefix).as_string() + suffix, update,
      std::accumulate(update.begin(), update.end(), 0),
      get_parameter(prefix + "_differential").as_bool(),
      get_parameter(prefix + "_relative").as_bool(), child_frame, rejection);
  }

  void scan_receipts()
  {
    while (!receipts_.empty()) {
      const auto receipt = receipts_.begin();
      if (receipt->second && !inputs_.count({receipt->first, 1})) {return;}
      scan_watermark_ns_ = std::max(scan_watermark_ns_, receipt->first);
      receipts_.erase(receipt);
    }
  }

  void drain()
  {
    const auto logical = next_index_ * period_ns_;
    const auto tick = (logical + 999999) / 1000000 * 1000000;
    const auto current = now().nanoseconds();
    if (current < tick) {return;}
    if (current > tick) {throw std::runtime_error("lockstep timeout: EKF missed exact update tick");}
    // Scan processing requires carried raw odometry after the scan. A fixed
    // 14 ms budget keeps current-scan arrival out of the filter's input set.
    const auto scan_cutoff = tick - 14000000;
    const auto expected_scan = scan_cutoff / 100000000 * 100000000;
    if (watermark_ns_ < tick || (expected_scan >= 100000000 && scan_watermark_ns_ < expected_scan)) {
      return;
    }
    for (auto input = inputs_.begin(); input != inputs_.end();) {
      const auto [stamp, index] = input->first;
      if (stamp > tick) {break;}
      if (index == 1 && stamp > scan_cutoff) {++input; continue;}
      odometryCallback(input->second, topics_[index], pose_data_[index], twist_data_[index]);
      input = inputs_.erase(input);
    }
    periodicUpdate();
    std_msgs::msg::Header ack;
    ack.stamp = rclcpp::Time(tick, RCL_ROS_TIME);
    update_ack_->publish(ack);
    ++next_index_;
  }

  std::string topics_[2];
  std::vector<robot_localization::CallbackData> pose_data_, twist_data_;
  std::map<std::pair<int64_t, size_t>, nav_msgs::msg::Odometry::SharedPtr> inputs_;
  std::map<int64_t, bool> receipts_;
  int64_t period_ns_ = 33333333, next_index_ = 1, watermark_ns_ = 0, scan_watermark_ns_ = 0;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr input_ack_, update_ack_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr retry_;
};
}  // namespace sentry_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto filter = std::make_shared<sentry_localization::LockstepEkf>();
  filter->start();
  rclcpp::spin(filter);
  rclcpp::shutdown();
  return 0;
}
