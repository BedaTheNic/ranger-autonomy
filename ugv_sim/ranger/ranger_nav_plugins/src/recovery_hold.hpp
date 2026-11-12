// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include "nav_msgs/msg/path.hpp"

namespace ranger_nav_plugins
{
// Status is tied to the exact retained path. An empty pose list clears it; the
// first reference pose carries a fresh simulation timestamp, not a new goal.
struct RecoveryHold
{
  using Clock = std::chrono::steady_clock;
  std_msgs::msg::Header path;
  Clock::time_point received{}, started{};
  double source_time{0.0};
  bool active{false};

  void update(const nav_msgs::msg::Path & status, Clock::time_point now)
  {
    if (status.poses.empty()) {active = false; return;}
    if (!active || path != status.header) {started = now;}
    active = true;
    path = status.header;
    received = now;
    const auto & stamp = status.poses.front().header.stamp;
    source_time = stamp.sec + stamp.nanosec * 1e-9;
  }

  bool permits(const std_msgs::msg::Header & requested, double ros_now,
    Clock::time_point now = Clock::now()) const
  {
    const double age = std::chrono::duration<double>(now - received).count();
    const double duration = std::chrono::duration<double>(now - started).count();
    return active && path == requested && !path.frame_id.empty() &&
      (path.stamp.sec != 0 || path.stamp.nanosec != 0) &&
      age >= 0.0 && age <= .4 && duration >= 0.0 && duration <= 26.0 &&
      source_time > 0.0 && ros_now >= source_time - .02 && ros_now - source_time <= .4;
  }
};
}  // namespace ranger_nav_plugins
