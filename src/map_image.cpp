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

#include "sentry_localization/map_image.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace sentry_localization
{

std::string to_pgm(const std::vector<int8_t> & data, uint32_t width, uint32_t height)
{
  if (data.size() != static_cast<size_t>(width) * height) {
    throw std::invalid_argument("map data size does not match width x height");
  }
  const auto free_max = std::lround(FREE_THRESH * 100);
  const auto occ_min = std::lround(OCCUPIED_THRESH * 100);
  std::string out = "P5\n" + std::to_string(width) + " " + std::to_string(height) + "\n255\n";
  out.reserve(out.size() + data.size());
  for (uint32_t row = 0; row < height; ++row) {
    const size_t src = static_cast<size_t>(height - 1 - row) * width;
    for (uint32_t col = 0; col < width; ++col) {
      const int v = data[src + col];
      unsigned char px = 205;
      if (v >= 0 && v <= 100) {
        if (v <= free_max) {
          px = 254;
        } else if (v >= occ_min) {
          px = 0;
        }
      }
      out.push_back(static_cast<char>(px));
    }
  }
  return out;
}

std::string to_yaml(
  const std::string & image, double resolution, double x, double y, double qz, double qw)
{
  const double yaw = 2.0 * std::atan2(qz, qw);
  char buf[256];
  // %.6g matches Python's {:.6g}; the thresholds print via {} (shortest repr).
  std::snprintf(
    buf, sizeof(buf),
    "\nmode: trinary\nresolution: %.6g\norigin: [%.6g, %.6g, %.6g]\nnegate: 0\n"
    "occupied_thresh: 0.65\nfree_thresh: 0.196\n",
    resolution, x, y, yaw);
  static_assert(OCCUPIED_THRESH == 0.65 && FREE_THRESH == 0.196, "update the yaml literals");
  return "image: " + image + buf;
}

void write_atomic(const std::string & path, const std::string & content)
{
  const std::string tmp = path + ".tmp";
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0) {
    throw std::runtime_error("open " + tmp + ": " + std::strerror(errno));
  }
  size_t done = 0;
  while (done < content.size()) {
    const ssize_t n = ::write(fd, content.data() + done, content.size() - done);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      const int e = errno;
      ::close(fd);
      throw std::runtime_error("write " + tmp + ": " + std::strerror(e));
    }
    done += static_cast<size_t>(n);
  }
  if (::fsync(fd) != 0) {
    const int e = errno;
    ::close(fd);
    throw std::runtime_error("fsync " + tmp + ": " + std::strerror(e));
  }
  ::close(fd);
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    throw std::runtime_error("rename " + tmp + ": " + std::strerror(errno));
  }
}

}  // namespace sentry_localization
