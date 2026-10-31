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
#include "sentry_localization/navigation_offset.hpp"

TEST(EkfInput, CarriesTwistAndStampWithoutFusingTheRawPosition)
{
  nav_msgs::msg::Odometry raw;
  raw.header.stamp.sec = 12;
  raw.header.stamp.nanosec = 345;
  raw.header.frame_id = "odom";
  raw.child_frame_id = "root";
  raw.pose.pose.position.x = -3.53;
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

TEST(NavigationOffset, WaitsForValidTeamThenOffsetsBothRawAndScanPositions)
{
  for (const bool blue : {false, true}) {
    sentry_localization::NavigationOffset offset;
    nav_msgs::msg::Odometry raw;
    raw.header.stamp.sec = 12;
    raw.child_frame_id = "root";
    raw.pose.pose.orientation.w = 1.0;
    raw.twist.twist.linear.y = 0.2;
    raw.pose.covariance[0] = 0.01;
    EXPECT_FALSE(offset.apply(raw, "nav_odom"));
    offset.observe(0, blue);
    EXPECT_FALSE(offset.apply(raw, "nav_odom"));
    offset.observe(7, blue);
    auto navigation = offset.apply(raw, "nav_odom");
    ASSERT_TRUE(navigation);
    EXPECT_DOUBLE_EQ(navigation->pose.pose.position.x, blue ? 3.53 : -3.53);
    EXPECT_DOUBLE_EQ(navigation->pose.pose.position.y, -0.426);
    raw.pose.pose.position.x = 0.25;
    raw.pose.pose.position.y = -0.15;
    offset.observe(7, !blue);  // This run keeps its first assignment.
    navigation = offset.apply(raw, "nav_odom");
    EXPECT_DOUBLE_EQ(navigation->pose.pose.position.x, (blue ? 3.53 : -3.53) + 0.25);
    EXPECT_DOUBLE_EQ(navigation->pose.pose.position.y, -0.426 - 0.15);
    EXPECT_EQ(navigation->header.stamp, raw.header.stamp);
    EXPECT_EQ(navigation->header.frame_id, "nav_odom");
    EXPECT_EQ(navigation->child_frame_id, raw.child_frame_id);
    EXPECT_EQ(navigation->twist, raw.twist);
    EXPECT_EQ(navigation->pose.covariance, raw.pose.covariance);
    EXPECT_EQ(navigation->pose.pose.orientation, raw.pose.pose.orientation);
    EXPECT_DOUBLE_EQ(raw.pose.pose.position.x, 0.25);
  }
}

TEST(NavigationOffset, RejectsNonzeroBootButAllowsTravelAfterZeroBoot)
{
  sentry_localization::NavigationOffset offset;
  offset.observe(7, true);
  nav_msgs::msg::Odometry raw;
  raw.pose.pose.position.x = 3.53;
  EXPECT_FALSE(offset.apply(raw, "nav_odom"));
  raw.pose.pose.position.x = 0.0;
  ASSERT_TRUE(offset.apply(raw, "nav_odom"));
  raw.pose.pose.position.x = 2.0;
  EXPECT_DOUBLE_EQ(offset.apply(raw, "nav_odom")->pose.pose.position.x, 5.53);
}

TEST(NavigationOffset, RoomWorkflowKeepsZeroOffsetWithoutReferee)
{
  sentry_localization::NavigationOffset offset(false);
  nav_msgs::msg::Odometry raw;
  raw.pose.pose.position.x = 0.6;
  auto navigation = offset.apply(raw, "nav_odom");
  ASSERT_TRUE(navigation);
  EXPECT_DOUBLE_EQ(navigation->pose.pose.position.x, 0.6);
  offset.observe(7, true);
  EXPECT_DOUBLE_EQ(offset.apply(raw, "nav_odom")->pose.pose.position.x, 0.6);
}
