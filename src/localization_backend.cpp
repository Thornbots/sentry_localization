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

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_amcl/amcl_node.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "nav2_map_server/map_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory_template.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

// nav2_amcl caches each scan frame's base->laser x/y at its first scan. Our
// lidar rides the head, about 0.22 m off root's yaw axis, so refresh that
// mount from TF at every scan stamp before AMCL's own callback. AMCL already
// re-reads the mount yaw per scan. See README.md (config/amcl.yaml).
//
// Lockstep simulation only (`scan_result_topic`): one result per scan, stamped
// with it: the pose AMCL published for that scan, or an empty frame_id for none.
// `refresh_after_s` replaces the nomotion service with a scan-keyed refresh, and
// `start_at_s` starts the filter from `start_pose` at that scan with `pf_seed`.
class TrackedMountAmcl : public nav2_amcl::AmclNode
{
public:
  explicit TrackedMountAmcl(const rclcpp::NodeOptions & options)
  : nav2_amcl::AmclNode(options)
  {
    const auto topic = declare_parameter("scan_result_topic", std::string(""));
    refresh_after_s_ = declare_parameter("refresh_after_s", 0.0);
    start_at_s_ = declare_parameter("start_at_s", 0.0);
    start_pose_ = declare_parameter("start_pose", std::vector<double>{});
    start_cov_ = declare_parameter("start_cov", std::vector<double>{0.25, 0.25, 0.07});
    pf_seed_ = declare_parameter("pf_seed", 1);
    // tf2 interpolates a sample at the query 1 ulp off once a newer one arrives; wait for one.
    tf_lookahead_s_ = declare_parameter("tf_lookahead_s", 0.0);
    if (start_at_s_ > 0.0 && (start_pose_.size() != 3 || start_cov_.size() != 3)) {
      throw std::invalid_argument("start_at_s needs start_pose and start_cov as [x, y, yaw]");
    }
    if (topic.empty()) {return;}
    // Plain publisher and subscription, not the message filter: every scan is answered
    // once, in stamp order, inactive or not, when TF covers it (retried on a wall timer).
    // Reliable and transient-local (the sim latches /scan): map_pose_publisher holds a tick
    // for each scan's answer, so a scan dropped here or sent before discovery matched
    // would stop the lockstep sim.
    result_pub_ = rclcpp::create_publisher<Pose>(
      get_node_topics_interface(), topic, rclcpp::QoS(100).reliable());
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      get_parameter("scan_topic").as_string(), rclcpp::QoS(100).reliable().transient_local(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        queue_.push_back(scan);
        pump();
      });
    pump_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {pump();});
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = nav2_amcl::AmclNode::on_configure(state);
    if (result_pub_) {laser_scan_connection_.disconnect();}  // scans come through pump()
    return result;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = nav2_amcl::AmclNode::on_activate(state);
    laser_scan_connection_.disconnect();
    if (!result_pub_) {
      laser_scan_connection_ = laser_scan_filter_->registerCallback(
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {receive(scan);});
    }
    return result;
  }

private:
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct StartDraw
  {
    std::mt19937_64 rng;
    std::normal_distribution<double> normal;
    std::vector<double> mean, var;

    static pf_vector_t sample(void * arg)
    {
      auto & d = *static_cast<StartDraw *>(arg);
      pf_vector_t v = pf_vector_zero();
      for (int i = 0; i < 3; ++i) {v.v[i] = d.mean[i] + std::sqrt(d.var[i]) * d.normal(d.rng);}
      return v;
    }
  };

  void publish_result(const builtin_interfaces::msg::Time & stamp, bool posed)
  {
    if (!result_pub_) {return;}
    Pose out = posed ? last_published_pose_ : Pose{};
    out.header.stamp = stamp;
    if (!posed) {out.header.frame_id.clear();}
    result_pub_->publish(out);
  }

  // Runs on this node's executor thread only; TF is checked without holding mutex_.
  void pump()
  {
    while (!queue_.empty()) {
      const auto scan = queue_.front();
      const rclcpp::Time stamp(scan->header.stamp);
      const bool skip = !active_ || (start_at_s_ > 0.0 && stamp.seconds() < start_at_s_);
      if (!skip && !tf_buffer_->canTransform(odom_frame_id_, scan->header.frame_id,
          tf2_ros::fromMsg(stamp + rclcpp::Duration::from_seconds(tf_lookahead_s_))))
      {
        return;  // TF at the scan comes with a later EKF output
      }
      queue_.pop_front();
      if (skip) {
        publish_result(scan->header.stamp, false);
      } else {
        receive(scan);
      }
    }
  }

  void receive(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    const rclcpp::Time stamp(scan->header.stamp);
    if (start_at_s_ > 0.0 && !started_) {
      if (stamp.seconds() < start_at_s_ || !pf_ || !map_) {
        publish_result(scan->header.stamp, false);
        return;
      }
      start(stamp);
    }
    if (refresh_after_s_ > 0.0) {
      const rclcpp::Time last(last_published_pose_.header.stamp, stamp.get_clock_type());
      if ((!refreshed_ || (stamp - *refreshed_).seconds() >= refresh_after_s_) &&
        (last.nanoseconds() == 0 || (stamp - last).seconds() >= refresh_after_s_))
      {
        force_update_ = true;
        refreshed_ = stamp;
      }
    }
    const auto before = last_published_pose_.header.stamp;
    process(scan);
    const auto & after = last_published_pose_.header.stamp;
    publish_result(scan->header.stamp, after != before && after == scan->header.stamp);
  }

  // The same particles at the same scan every run, whenever activation and the wall-timed
  // /initialpose happened; later /initialpose messages are ignored.
  void start(const rclcpp::Time & stamp)
  {
    // Not pf_init: its Gaussian reseeds drand48 from a count of earlier /initialpose.
    StartDraw draw{std::mt19937_64(static_cast<uint64_t>(pf_seed_)), {}, start_pose_, start_cov_};
    pf_init_model(pf_, &StartDraw::sample, &draw);
    srand48(pf_seed_);  // motion noise and resampling
    pf_init_ = false;
    initial_pose_is_known_ = true;
    resample_count_ = 0;
    initial_pose_sub_.reset();
    started_ = true;
    RCLCPP_INFO(get_logger(), "AMCL started at scan %.3f s, seed %s", stamp.seconds(),
      std::to_string(pf_seed_).c_str());
    if (stamp.seconds() > start_at_s_ + 0.2) {
      RCLCPP_ERROR(get_logger(), "AMCL started late, at scan %.3f s (start_at_s %.3f)",
        stamp.seconds(), start_at_s_);
    }
  }

  void process(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
  {
    if (result_pub_ && frame_to_laser_.count(scan->header.frame_id) == 0) {
      // AMCL registers a laser from latest TF; register it here so its first scan's
      // mount comes from the scan stamp too.
      int index = 0;
      geometry_msgs::msg::PoseStamped laser_pose;
      if (!addNewScanner(index, scan, scan->header.frame_id, laser_pose)) {return;}
    }
    const auto found = frame_to_laser_.find(scan->header.frame_id);
    if (found != frame_to_laser_.end()) {
      std::string frame = scan->header.frame_id;
      if (!frame.empty() && frame.front() == '/') {
        frame.erase(0, 1);
      }
      geometry_msgs::msg::TransformStamped mount;
      try {
        mount = tf_buffer_->lookupTransform(
          base_frame_id_, frame, tf2_ros::fromMsg(scan->header.stamp));
      } catch (const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Skipping scan without stamped %s->%s: %s",
          base_frame_id_.c_str(), frame.c_str(), error.what());
        return;
      }
      pf_vector_t pose = pf_vector_zero();
      pose.v[0] = mount.transform.translation.x;
      pose.v[1] = mount.transform.translation.y;
      lasers_[found->second]->SetLaserPose(pose);
    }
    laserReceived(scan);
  }

  double refresh_after_s_{0.0}, start_at_s_{0.0}, tf_lookahead_s_{0.0};
  std::vector<double> start_pose_, start_cov_;
  int64_t pf_seed_{1};
  bool started_{false};
  std::optional<rclcpp::Time> refreshed_;
  std::deque<sensor_msgs::msg::LaserScan::ConstSharedPtr> queue_;
  rclcpp::TimerBase::SharedPtr pump_timer_;
  rclcpp::Publisher<Pose>::SharedPtr result_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

// A new mapping session starts at map zero even when nav_odom is field-offset.
// Saved graphs keep slam_toolbox's existing map_start_pose registration.
class ZeroOriginSlam : public slam_toolbox::AsynchronousSlamToolbox
{
public:
  explicit ZeroOriginSlam(rclcpp::NodeOptions options)
  : slam_toolbox::AsynchronousSlamToolbox(options)
  {
    declare_parameter("zero_origin_mapping", false);
  }

protected:
  karto::LocalizedRangeScan * addScan(
    karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan, karto::Pose2 & odom_pose) override
  {
    if (!origin_initialized_ && get_parameter("zero_origin_mapping").as_bool()) {
      reprocessing_transform_ = smapper_->toTfPose(odom_pose).inverse();
      origin_initialized_ = true;
    }
    return slam_toolbox::SlamToolbox::addScan(laser, scan, odom_pose);
  }

private:
  bool origin_initialized_{false};
};

// Static composition keeps lifecycle services local and libraries loaded until exit.
// Each node has its own executor; see README.md.
class Backend
{
public:
  ~Backend()
  {
    for (const auto & executor : executors_) {
      executor->cancel();
    }
    for (auto & thread : threads_) {
      thread.join();
    }
    executors_.clear();
    while (!nodes_.empty()) {
      nodes_.pop_back();
    }
  }

  template<typename NodeT>
  void add(const std::string & name)
  {
    // Local name remaps override the process name; YAML retains its node-name scopes.
    auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=" + name});
    rclcpp_components::NodeFactoryTemplate<NodeT> factory;
    auto node = factory.create_node_instance(options);
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node.get_node_base_interface());
    nodes_.push_back(std::move(node));
    executors_.push_back(executor);
  }

  void spin()
  {
    for (size_t i = 0; i + 1 < executors_.size(); ++i) {
      threads_.emplace_back([executor = executors_[i]] {executor->spin();});
    }
    executors_.back()->spin();
  }

private:
  std::vector<rclcpp_components::NodeInstanceWrapper> nodes_;
  std::vector<std::shared_ptr<rclcpp::Executor>> executors_;
  std::vector<std::thread> threads_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  Backend backend;
  int result = 0;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() != 2 || (args[1] != "amcl" && args[1] != "slam")) {
      throw std::invalid_argument("Expected backend argument: amcl or slam");
    }
    if (args[1] == "amcl") {
      backend.add<nav2_map_server::MapServer>("map_server");
      backend.add<TrackedMountAmcl>("amcl");
    } else {
      backend.add<ZeroOriginSlam>("slam_toolbox");
    }
    backend.add<nav2_lifecycle_manager::LifecycleManager>(
      args[1] == "amcl" ? "lifecycle_manager_localization" : "lifecycle_manager_slam");
    backend.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("localization_backend"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
