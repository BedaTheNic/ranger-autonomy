// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "geometry_msgs/msg/pose_stamped.hpp"
#ifdef RANGER_NAV2_JAZZY
#include "nav2_core/controller_exceptions.hpp"
#else
#include "nav2_core/exceptions.hpp"
#endif

namespace ranger_nav_plugins
{
#ifdef RANGER_NAV2_JAZZY
using AckermannTrackingError = nav2_core::ControllerException;
#else
using AckermannTrackingError = nav2_core::PlannerException;
#endif

// In a pure Ackermann profile this asks Nav2 for a new approach. The
// adaptive controller can instead align at the cusp on the retained path.
struct ReversalAlignment : AckermannTrackingError
{
  explicit ReversalAlignment(const geometry_msgs::msg::PoseStamped & cusp)
  : AckermannTrackingError("Ackermann reversal requires a new approach to the cusp pose"), target(cusp) {}
  geometry_msgs::msg::PoseStamped target;
};
}
