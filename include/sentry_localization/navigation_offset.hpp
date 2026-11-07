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
#include <stdexcept>
#include <string>
#include "nav_msgs/msg/odometry.hpp"

namespace sentry_localization
{
struct TeamStarts
{
  double red_x, red_y, blue_x, blue_y;
};

// Latch the first valid referee team for this run; never guess from a zero ID.
class NavigationOffset
{
public:
  explicit NavigationOffset(
    TeamStarts starts, bool require_team = true, const std::string & explicit_team = "")
  : starts_(starts), require_zero_start_(require_team || !explicit_team.empty())
  {
    if (!std::isfinite(starts.red_x) || !std::isfinite(starts.red_y) ||
      !std::isfinite(starts.blue_x) || !std::isfinite(starts.blue_y))
    {
      throw std::invalid_argument("Team starts must be finite field coordinates");
    }
    if (!require_team) {
      start_x_ = explicit_team.empty() ? 0.0 :
        (explicit_team == "blue" ? starts_.blue_x : starts_.red_x);
      start_y_ = explicit_team.empty() ? 0.0 :
        (explicit_team == "blue" ? starts_.blue_y : starts_.red_y);
    }
  }

  void observe(unsigned int robot_id, bool blue)
  {
    if (robot_id != 0 && !start_x_) {
      start_x_ = blue ? starts_.blue_x : starts_.red_x;
      start_y_ = blue ? starts_.blue_y : starts_.red_y;
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
  TeamStarts starts_;
  bool require_zero_start_;
  bool started_{false};
  std::optional<double> start_x_;
  double start_y_{0.0};
};
}  // namespace sentry_localization
#endif  // SENTRY_LOCALIZATION__NAVIGATION_OFFSET_HPP_
