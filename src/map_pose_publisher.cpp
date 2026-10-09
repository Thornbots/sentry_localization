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

// Publish /localization/odom moved into the map frame, on /localization/map_odom.
//
// Pose: map->odom (TF, at the input's stamp, else the latest) composed with
// the input's odom->root. Stamp and twist (child frame) are the input's.
// xy covariance: the input's, rotated into map, plus the latest xy
// covariance on backend_pose_topic (/amcl_pose or slam_toolbox's /pose).
// Not launched under localization_mode:=none, which has no map frame.
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sentry_localization/map_pose_core.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using sentry_localization::Cov2;
using sentry_localization::Pose2D;

namespace
{

// xy variance (m^2) published until the backend's first pose arrives, so
// mcb_relay's max_std_m gate refuses to relocalize from it.
constexpr double UNKNOWN_VAR = 1.0;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

class MapPosePublisher : public rclcpp::Node
{
public:
  MapPosePublisher()
  : Node("map_pose_publisher")
  {
    declare_parameter<std::string>("input_topic", "/localization/odom");
    declare_parameter<std::string>("output_topic", "/localization/map_odom");
    declare_parameter<std::string>("backend_pose_topic", "/amcl_pose");
    declare_parameter<std::string>("map_frame", "map");
    map_frame_ = get_parameter("map_frame").as_string();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    pub_ = create_publisher<nav_msgs::msg::Odometry>(get_parameter("output_topic").as_string(), 10);
    backend_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("backend_pose_topic").as_string(), 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        const auto & c = msg->pose.covariance;
        backend_cov_ = Cov2{{{c[0], c[1]}, {c[6], c[7]}}};
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("input_topic").as_string(), 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odom_cb(*msg);});
  }

private:
  std::optional<Pose2D> map_odom(const std::string & odom_frame, const builtin_interfaces::msg::Time & stamp)
  {
    const tf2::TimePoint times[] = {
      tf2_ros::fromMsg(stamp), tf2::TimePointZero};
    for (const auto & t : times) {
      try {
        const auto tf = tf_buffer_->lookupTransform(map_frame_, odom_frame, t);
        const auto & tr = tf.transform.translation;
        return Pose2D{tr.x, tr.y, yaw_of(tf.transform.rotation)};
      } catch (const tf2::TransformException &) {
        continue;
      }
    }
    return std::nullopt;
  }

  void odom_cb(const nav_msgs::msg::Odometry & msg)
  {
    const auto mo = map_odom(msg.header.frame_id, msg.header.stamp);
    if (!mo) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no %s->%s yet; not publishing",
        map_frame_.c_str(), msg.header.frame_id.c_str());
      return;
    }
    const auto & p = msg.pose.pose;
    const Pose2D pose = sentry_localization::compose(*mo, {p.position.x, p.position.y, yaw_of(p.orientation)});
    const auto & c = msg.pose.covariance;
    const Cov2 rot = sentry_localization::rotate_cov_xy(Cov2{{{c[0], c[1]}, {c[6], c[7]}}}, mo->yaw);
    const Cov2 b = backend_cov_.value_or(Cov2{{{UNKNOWN_VAR, 0.0}, {0.0, UNKNOWN_VAR}}});

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg.header.stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = msg.child_frame_id;
    out.pose.pose.position.x = pose.x;
    out.pose.pose.position.y = pose.y;
    out.pose.pose.position.z = p.position.z;
    out.pose.pose.orientation.z = std::sin(pose.yaw / 2.0);
    out.pose.pose.orientation.w = std::cos(pose.yaw / 2.0);
    out.pose.covariance = c;
    out.pose.covariance[0] = rot[0][0] + b[0][0];
    out.pose.covariance[1] = rot[0][1] + b[0][1];
    out.pose.covariance[6] = rot[0][1] + b[0][1];
    out.pose.covariance[7] = rot[1][1] + b[1][1];
    out.twist = msg.twist;
    pub_->publish(out);
  }

  std::string map_frame_;
  std::optional<Cov2> backend_cov_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr backend_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
