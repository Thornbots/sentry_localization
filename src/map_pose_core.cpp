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

#include "sentry_localization/map_pose_core.hpp"

#include <cmath>

namespace sentry_localization
{

Pose2D compose(const Pose2D & map_odom, const Pose2D & odom_root)
{
  const double c = std::cos(map_odom.yaw);
  const double s = std::sin(map_odom.yaw);
  const double sum = map_odom.yaw + odom_root.yaw;
  return {
    map_odom.x + c * odom_root.x - s * odom_root.y,
    map_odom.y + s * odom_root.x + c * odom_root.y,
    std::atan2(std::sin(sum), std::cos(sum))};
}

Cov2 rotate_cov_xy(const Cov2 & cov_xy, double yaw)
{
  const double a = cov_xy[0][0];
  const double b = cov_xy[0][1];
  const double d = cov_xy[1][1];
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double xx = c * c * a - 2 * c * s * b + s * s * d;
  const double yy = s * s * a + 2 * c * s * b + c * c * d;
  const double xy = c * s * (a - d) + (c * c - s * s) * b;
  return {{{xx, xy}, {xy, yy}}};
}

}  // namespace sentry_localization
