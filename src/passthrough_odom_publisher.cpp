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

// Relays /odom onto /localization/odom unchanged.
//
// Used for localization_mode in {slam, mapping, amcl}, where root pose is
// uncorrected wheel odometry (only map->odom is corrected, by slam_toolbox/
// amcl directly). Not launched for localization_mode:=ekf, which publishes
// /localization/odom itself instead.
#include <memory>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PassthroughOdomPublisher : public rclcpp::Node
{
public:
  PassthroughOdomPublisher()
  : Node("passthrough_odom_publisher")
  {
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) {pub_->publish(std::move(msg));});
  }

private:
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
