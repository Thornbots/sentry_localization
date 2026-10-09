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

// OccupancyGrid to nav2 map_server .pgm/.yaml, no rclcpp (map_autosaver).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sentry_localization
{

constexpr double FREE_THRESH = 0.196;  // nav2 Jazzy map_saver defaults
constexpr double OCCUPIED_THRESH = 0.65;

// P5 bytes as nav2 map_saver's trinary mode writes them, top row = max y.
// Throws std::invalid_argument if data.size() != width * height.
std::string to_pgm(const std::vector<int8_t> & data, uint32_t width, uint32_t height);

// map_server yaml for an image next to it; origin yaw from the quaternion.
std::string to_yaml(
  const std::string & image, double resolution, double x, double y, double qz, double qw);

// Write via a temp file and rename, so a power cut leaves the old file.
// Throws std::runtime_error on I/O failure.
void write_atomic(const std::string & path, const std::string & content);

}  // namespace sentry_localization
