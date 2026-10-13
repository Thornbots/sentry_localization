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

// With use_rf2o=false, navigation equals raw odometry in the same aligned axes.
// Publish that navigation pose as nav_odom; the raw /odom stream stays untouched.
#include <memory>
#include <string>
#include <utility>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class PassthroughOdomPublisher : public rclcpp::Node
{
public:
  PassthroughOdomPublisher()
  : Node("passthrough_odom_publisher")
  {
    navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::UniquePtr msg) {
        msg->header.frame_id = navigation_frame_;
        pub_->publish(std::move(msg));
      });
  }

private:
  std::string navigation_frame_;
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
