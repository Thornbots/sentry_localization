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

// Navigation adds the referee team start to zero-origin, field-aligned odometry.
// Publish that navigation pose as nav_odom; the raw /odom stream stays untouched.
#include <memory>
#include <string>
#include <utility>

#include "dji_serial_bridge/msg/ref_sys_status.hpp"
#include "sentry_localization/navigation_offset.hpp"
#include "rclcpp/rclcpp.hpp"

class PassthroughOdomPublisher : public rclcpp::Node
{
public:
  PassthroughOdomPublisher()
  : Node("passthrough_odom_publisher")
  {
    offset_ = sentry_localization::NavigationOffset(
      declare_parameter("require_team", true),
      declare_parameter("explicit_team", std::string("")));
    navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    team_sub_ = create_subscription<dji_serial_bridge::msg::RefSysStatus>(
      "/dji_serial_bridge/ref_sys", rclcpp::SensorDataQoS(),
      [this](dji_serial_bridge::msg::RefSysStatus::ConstSharedPtr msg) {
        offset_.observe(msg->robot_id, msg->is_on_blue_team);
      });
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        const auto navigation = offset_.apply(*msg, navigation_frame_);
        if (navigation) {
          pub_->publish(*navigation);
        }
      });
  }

private:
  sentry_localization::NavigationOffset offset_;
  std::string navigation_frame_;
  rclcpp::Subscription<dji_serial_bridge::msg::RefSysStatus>::SharedPtr team_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PassthroughOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
