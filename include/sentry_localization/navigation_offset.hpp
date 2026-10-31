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
#ifndef SENTRY_LOCALIZATION__NAVIGATION_OFFSET_HPP_
#define SENTRY_LOCALIZATION__NAVIGATION_OFFSET_HPP_

#include <cmath>
#include <optional>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

namespace sentry_localization
{
// TEMPORARY spooky scrim start (unmeasured, not good); restore or measure before the next
// event, see docs/ROADMAP.md. Matches firmware START_X/START_Y; y is not mirrored.
constexpr double kStartX = 3.53;
constexpr double kStartY = -0.426;

// Latch the first valid referee team for this run; never guess from a zero ID.
class NavigationOffset
{
public:
  explicit NavigationOffset(bool require_team = true, const std::string & explicit_team = "")
  : require_zero_start_(require_team || !explicit_team.empty())
  {
    if (!require_team) {
      start_x_ = explicit_team.empty() ? 0.0 : (explicit_team == "blue" ? kStartX : -kStartX);
      start_y_ = explicit_team.empty() ? 0.0 : kStartY;
    }
  }

  void observe(unsigned int robot_id, bool blue)
  {
    if (robot_id != 0 && !start_x_) {
      start_x_ = blue ? kStartX : -kStartX;
      start_y_ = kStartY;
    }
  }

  std::optional<nav_msgs::msg::Odometry> apply(
    const nav_msgs::msg::Odometry & raw, const std::string & frame)
  {
    if (!start_x_) {
      return std::nullopt;
    }
    const auto & position = raw.pose.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      (!started_ && require_zero_start_ && std::hypot(position.x, position.y) > 0.3))
    {
      return std::nullopt;
    }
    started_ = true;
    auto out = raw;
    out.header.frame_id = frame;
    out.pose.pose.position.x += *start_x_;
    out.pose.pose.position.y += start_y_;
    return out;
  }

private:
  bool require_zero_start_;
  bool started_{false};
  std::optional<double> start_x_;
  double start_y_{0.0};
};
}  // namespace sentry_localization
#endif  // SENTRY_LOCALIZATION__NAVIGATION_OFFSET_HPP_
