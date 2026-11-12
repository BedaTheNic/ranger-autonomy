// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "navigation_geometry.hpp"

#ifdef RANGER_NAV2_JAZZY
#include "nav2_core/controller_exceptions.hpp"
#else
#include "nav2_core/exceptions.hpp"
#endif

namespace ranger_nav_plugins
{
#ifdef RANGER_NAV2_JAZZY
using TrackingException = nav2_core::ControllerException;
#else
using TrackingException = nav2_core::PlannerException;
#endif

struct FootprintBlockage
{
  Pose2 target;
  bool rotate;
};

class FootprintBlocked : public TrackingException, public FootprintBlockage
{
public:
  FootprintBlocked(const char * message, const Pose2 & target, bool rotate = false)
  : TrackingException(message), FootprintBlockage{target, rotate} {}
};

// Caller holds the local costmap mutex. Expand each occupied cell to its actual
// square and check the complete translational sweep. The footprint's edge
// normals and the sweep normal are sufficient for a convex footprint. With a
// nonconvex footprint the projected sweep is conservative.
inline bool escapeTranslationFree(
  const Pose2 & start, const Pose2 & finish, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint, double padding = 0.0)
{
  if (std::abs(wrap(finish.a - start.a)) > 1e-6) {return false;}
  if (!pathFree({start, finish}, map, footprint)) {return false;}
  nav2_costmap_2d::Footprint polygon;
  nav2_costmap_2d::transformFootprint(start.x, start.y, start.a, footprint, polygon);
  const double dx = finish.x - start.x, dy = finish.y - start.y;
  struct Axis {double x, y, lower, upper;};
  std::vector<Axis> axes;
  auto add_axis = [&](double x, double y) {
      if (x == 0.0 && y == 0.0) {return;}
      double lower = INFINITY, upper = -INFINITY;
      for (const auto & vertex : polygon) {
        const double projection = x * vertex.x + y * vertex.y;
        lower = std::min(lower, projection);
        upper = std::max(upper, projection);
      }
      const double shift = x * dx + y * dy;
      const double margin = padding * std::hypot(x, y);
      axes.push_back({x, y,
        lower + std::min(0.0, shift) - margin, upper + std::max(0.0, shift) + margin});
    };
  add_axis(1.0, 0.0);
  add_axis(0.0, 1.0);
  add_axis(-dy, dx);
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    add_axis(a.y - b.y, b.x - a.x);
  }
  const double half_cell = map.getResolution() / 2.0;
  unsigned x0, y0, x1, y1;
  if (!map.worldToMap(axes[0].lower - half_cell, axes[1].lower - half_cell, x0, y0) ||
    !map.worldToMap(axes[0].upper + half_cell, axes[1].upper + half_cell, x1, y1))
  {
    return false;
  }
  for (unsigned y = y0; y <= y1; ++y) {
    for (unsigned x = x0; x <= x1; ++x) {
      if (map.getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
      double wx, wy;
      map.mapToWorld(x, y, wx, wy);
      const auto separates = [&](const Axis & axis) {
          const double center = axis.x * wx + axis.y * wy;
          const double radius = half_cell * (std::abs(axis.x) + std::abs(axis.y));
          return center + radius < axis.lower || center - radius > axis.upper;
        };
      if (std::none_of(axes.begin(), axes.end(), separates)) {return false;}
    }
  }
  return true;
}

inline bool escapeRotationFree(
  const Pose2 & pose, double heading, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  const double angle = wrap(heading - pose.a);
  if (!std::isfinite(angle)) {return false;}
  double radius = 0.0;
  for (const auto & vertex : footprint) {radius = std::max(radius, std::hypot(vertex.x, vertex.y));}
  const int count = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / .025)));
  const double step = angle / count;
  // At any orientation in the interval every footprint point lies within
  // this distance of its position at the midpoint, including the endpoints.
  const double padding = 2.0 * radius * std::sin(std::abs(step) / 4.0);
  for (int i = 0; i < count; ++i) {
    const Pose2 middle{pose.x, pose.y, pose.a + (i + .5) * step};
    if (!escapeTranslationFree(middle, middle, map, footprint, padding)) {return false;}
  }
  return true;
}

inline bool escapeRestoresTracking(
  const Pose2 & pose, const FootprintBlockage & blocked, nav2_costmap_2d::Costmap2D & map,
  const nav2_costmap_2d::Footprint & footprint)
{
  auto approach = pose;
  if (blocked.rotate) {
    if (!escapeRotationFree(pose, blocked.target.a, map, footprint)) {return false;}
    approach.a = blocked.target.a;
  }
  return escapeTranslationFree(
    approach, {blocked.target.x, blocked.target.y, approach.a}, map, footprint);
}

inline std::optional<Pose2> findFootprintEscape(
  const Pose2 & robot, const FootprintBlockage & blocked, double budget,
  nav2_costmap_2d::Costmap2D & map, const nav2_costmap_2d::Footprint & footprint)
{
  if (!escapeTranslationFree(robot, robot, map, footprint) ||
    escapeRestoresTracking(robot, blocked, map, footprint))
  {
    return std::nullopt;
  }
  // Prefer the shortest verified move. Probe all directions at each radius,
  // because adjacent or opposing obstacles can make a single normal unsafe.
  constexpr int directions = 16;
  for (double radius = 0.06; radius <= budget + 1e-6; radius += 0.06) {
    for (int i = 0; i < directions; ++i) {
      const double angle = robot.a + 2.0 * M_PI * i / directions;
      const Pose2 candidate{
        robot.x + radius * std::cos(angle), robot.y + radius * std::sin(angle), robot.a};
      if (escapeTranslationFree(robot, candidate, map, footprint) &&
        escapeRestoresTracking(candidate, blocked, map, footprint))
      {
        return candidate;
      }
    }
  }
  return std::nullopt;
}
}  // namespace ranger_nav_plugins
