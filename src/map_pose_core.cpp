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

Pose2D map_correction(const Pose2D & map_root, const Pose2D & nav_root)
{
  const double c = std::cos(nav_root.yaw), s = std::sin(nav_root.yaw);
  return compose(map_root, {-c*nav_root.x-s*nav_root.y,
    s*nav_root.x-c*nav_root.y, -nav_root.yaw});
}

bool usable_backend_covariance(const Cov2 & cov, double yaw_var,
  double backend_t, double pose_t, double max_age_s)
{
  const double a = cov[0][0], b = cov[0][1], d = cov[1][1];
  return std::isfinite(backend_t) && std::isfinite(pose_t) &&
    pose_t >= backend_t && pose_t - backend_t <= max_age_s &&
    std::isfinite(a) && std::isfinite(b) && std::isfinite(d) &&
    std::isfinite(cov[1][0]) && std::abs(b - cov[1][0]) < 1e-9 &&
    // AMCL's zero-spread prior can have roundoff near -1e-12 m^2.
    .5 * (a + d - std::hypot(a - d, 2*b)) >= -1e-9 &&
    std::isfinite(yaw_var) && yaw_var >= -1e-9;
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
