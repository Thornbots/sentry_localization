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

#include <cmath>

#include "sentry_localization/map_pose_core.hpp"

using sentry_localization::compose;
using sentry_localization::Cov2;
using sentry_localization::Pose2D;
using sentry_localization::rotate_cov_xy;

namespace
{
// pytest.approx defaults: rel 1e-6, abs 1e-12.
void expect_approx(double actual, double expected)
{
  EXPECT_NEAR(actual, expected, std::max(1e-6 * std::fabs(expected), 1e-12));
}
}  // namespace

TEST(MapPoseCore, IdentityMapOdomPassesPoseThrough)
{
  const Pose2D p = compose({0.0, 0.0, 0.0}, {1.0, -2.0, 0.3});
  expect_approx(p.x, 1.0);
  expect_approx(p.y, -2.0);
  expect_approx(p.yaw, 0.3);
}

TEST(MapPoseCore, TranslationAdds)
{
  const Pose2D p = compose({0.5, 0.25, 0.0}, {1.0, 1.0, 0.0});
  expect_approx(p.x, 1.5);
  expect_approx(p.y, 1.25);
  expect_approx(p.yaw, 0.0);
}

TEST(MapPoseCore, RotationTurnsOdomPositionThenTranslates)
{
  const Pose2D p = compose({1.0, 0.0, M_PI / 2}, {2.0, 0.0, 0.0});
  expect_approx(p.x, 1.0);
  expect_approx(p.y, 2.0);
  expect_approx(p.yaw, M_PI / 2);
}

TEST(MapPoseCore, YawWraps)
{
  expect_approx(compose({0.0, 0.0, 3.0}, {0.0, 0.0, 3.0}).yaw, 6.0 - 2 * M_PI);
}

TEST(MapPoseCore, CovRotationQuarterTurnSwapsAxes)
{
  const Cov2 r = rotate_cov_xy({{{4.0, 0.0}, {0.0, 1.0}}}, M_PI / 2);
  expect_approx(r[0][0], 1.0);
  expect_approx(r[0][1], 0.0);
  expect_approx(r[1][0], 0.0);
  expect_approx(r[1][1], 4.0);
}

TEST(MapPoseCore, CovRotationKeepsTraceAndSymmetry)
{
  const Cov2 r = rotate_cov_xy({{{3.0, 0.5}, {0.5, 2.0}}}, 0.7);
  expect_approx(r[0][0] + r[1][1], 5.0);
  EXPECT_EQ(r[0][1], r[1][0]);
  expect_approx(r[0][0] * r[1][1] - r[0][1] * r[0][1], 3.0 * 2.0 - 0.25);
}
