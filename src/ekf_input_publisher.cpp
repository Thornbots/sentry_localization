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
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sentry_localization/ekf_input.hpp"

class EkfInputPublisher : public rclcpp::Node
{
public:
  EkfInputPublisher()
  : Node("ekf_input_publisher")
  {
    navigation_frame_ = declare_parameter("navigation_frame", std::string("nav_odom"));
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/localization/ekf_input", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr raw) {
        pub_->publish(sentry_localization::ekf_input(*raw, navigation_frame_));
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
  rclcpp::spin(std::make_shared<EkfInputPublisher>());
  rclcpp::shutdown();
  return 0;
}
