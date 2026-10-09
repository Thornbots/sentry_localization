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
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sentry_localization/map_image.hpp"

using sentry_localization::to_pgm;
using sentry_localization::to_yaml;
using sentry_localization::write_atomic;

TEST(MapImage, PgmTrinaryValuesAndRowOrder)
{
  // Row 0 of the grid is min y, so it is the image's last row.
  const std::vector<int8_t> data = {-1, 0, 20, 21,
    64, 65, 100, 101};
  const std::string pgm = to_pgm(data, 4, 2);
  const std::string header = "P5\n4 2\n255\n";
  ASSERT_EQ(pgm.compare(0, header.size(), header), 0);
  const std::vector<unsigned char> expected = {205, 0, 0, 205,
    205, 254, 254, 205};
  ASSERT_EQ(pgm.size(), header.size() + expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(static_cast<unsigned char>(pgm[header.size() + i]), expected[i]) << i;
  }
}

TEST(MapImage, PgmRejectsSizeMismatch)
{
  EXPECT_THROW(to_pgm({0, 0, 0}, 2, 2), std::invalid_argument);
}

TEST(MapImage, YamlOriginYawFromQuaternion)
{
  const std::string y = to_yaml("map.pgm", 0.05, -1.5, 2.0, std::sin(0.25), std::cos(0.25));
  EXPECT_NE(y.find("image: map.pgm\n"), std::string::npos);
  EXPECT_NE(y.find("mode: trinary\n"), std::string::npos);
  EXPECT_NE(y.find("resolution: 0.05\n"), std::string::npos);
  EXPECT_NE(y.find("origin: [-1.5, 2, 0.5]\n"), std::string::npos);
  EXPECT_EQ(
    y,
    "image: map.pgm\nmode: trinary\nresolution: 0.05\norigin: [-1.5, 2, 0.5]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n");
}

TEST(MapImage, WriteAtomicReplacesAndLeavesNoTmp)
{
  const auto dir = std::filesystem::temp_directory_path() / "sentry_loc_map_image_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);
  const std::string p = (dir / "map.yaml").string();
  write_atomic(p, "old");
  write_atomic(p, "new");
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  EXPECT_EQ(ss.str(), "new");
  int n = 0;
  for (const auto & e : std::filesystem::directory_iterator(dir)) {
    EXPECT_EQ(e.path().filename(), "map.yaml");
    ++n;
  }
  EXPECT_EQ(n, 1);
  std::filesystem::remove_all(dir);
}
