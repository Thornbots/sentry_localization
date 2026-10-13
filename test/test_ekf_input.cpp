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
#include <gtest/gtest.h>
#include "sentry_localization/ekf_input.hpp"

TEST(EkfInput, CarriesTwistAndStampWithoutFusingTheRawPosition)
{
  nav_msgs::msg::Odometry raw;
  raw.header.stamp.sec = 12;
  raw.header.stamp.nanosec = 345;
  raw.header.frame_id = "odom";
  raw.child_frame_id = "root";
  raw.pose.pose.position.x = -4.625;
  raw.pose.pose.position.y = 1.25;
  raw.twist.twist.linear.x = 2.0;
  raw.twist.twist.linear.y = -1.0;
  raw.twist.covariance[0] = 0.0001;
  const auto input = sentry_localization::ekf_input(raw, "nav_odom");
  EXPECT_EQ(input.header.stamp, raw.header.stamp);
  EXPECT_EQ(input.header.frame_id, "nav_odom");
  EXPECT_EQ(input.child_frame_id, "root");
  EXPECT_EQ(input.twist, raw.twist);
  EXPECT_DOUBLE_EQ(input.pose.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(input.pose.pose.position.y, 0.0);
}

TEST(EkfInput, HeadingIsDefinedInNavigationAxesWithoutAnOutputTfDependency)
{
  nav_msgs::msg::Odometry raw;
  raw.pose.pose.orientation.z = 0.2;
  raw.pose.pose.orientation.w = 0.98;
  raw.pose.covariance[35] = 0.00001;
  const auto input = sentry_localization::ekf_input(raw, "custom_navigation");
  EXPECT_EQ(input.header.frame_id, "custom_navigation");
  EXPECT_DOUBLE_EQ(input.pose.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(input.pose.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(input.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(input.pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(input.pose.covariance[35], raw.pose.covariance[35]);
}
