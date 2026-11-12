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
#ifndef SENTRY_LOCALIZATION__EKF_INPUT_HPP_
#define SENTRY_LOCALIZATION__EKF_INPUT_HPP_

#include <string>
#include "nav_msgs/msg/odometry.hpp"

namespace sentry_localization
{
// Only yaw and root-frame vx/vy are fused; raw position never enters the EKF.
// Navigation axes share raw heading, so this fixed-heading measurement needs no TF.
inline nav_msgs::msg::Odometry ekf_input(
  const nav_msgs::msg::Odometry & raw, const std::string & navigation_frame)
{
  nav_msgs::msg::Odometry out;
  out.header = raw.header;
  out.header.frame_id = navigation_frame;
  out.child_frame_id = raw.child_frame_id;
  out.pose.pose.orientation.w = 1.0;
  out.pose.covariance[35] = raw.pose.covariance[35];
  out.twist = raw.twist;
  return out;
}
}  // namespace sentry_localization
#endif  // SENTRY_LOCALIZATION__EKF_INPUT_HPP_
