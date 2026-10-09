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

// Planar map->root composition for map_pose_publisher (no rclcpp).
#pragma once

#include <array>

namespace sentry_localization
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

using Cov2 = std::array<std::array<double, 2>, 2>;

// map->root from map->odom and odom->root; yaw wrapped to (-pi, pi].
Pose2D compose(const Pose2D & map_odom, const Pose2D & odom_root);

// Rotate a 2x2 xy covariance by yaw: R C R^T.
Cov2 rotate_cov_xy(const Cov2 & cov_xy, double yaw);

}  // namespace sentry_localization
