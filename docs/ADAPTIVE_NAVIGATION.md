# Adaptive navigation — Ranger Mini V2 simulation

The `adaptive` profile selects between Smac Hybrid-A* and Smac State
Lattice paths. MPPI controls Ackermann segments, while DWB controls
holonomic segments. The existing `ackermann` and `holonomic` profiles
remain available for comparison. These parameters apply to the Mini V2
simulation; they do not calibrate a larger Ranger.

Since 2026-11-02, the holonomic planner and controller support direct diagonal
motion, and global route selection prioritizes length. Change details and
validation: [DIAGONAL_NAVIGATION.md](DIAGONAL_NAVIGATION.md).

In the current simulation, `/is_retained_path_valid` checks the remaining
path using the same footprint as the planner. It logs the rejected path's
timestamp and first blocked pose. Hybrid-A* retains the raw path while
validating a smoothed copy. Long straight segments can receive a candidate
whose chassis heading follows the direction of travel, with both rotations,
required time, and clearance checked. The `navigation_motion_guard` node
checks the final command after limits and any smoothing. It uses the local
costmap, scan age, and measured velocity. Commands before and after this
check appear on `/cmd_vel_candidate` and `/cmd_vel`.
[Manual validation instructions, recorded drives, and results](validation/navigation-improvements-20261103/README.md).

## Starting a new world

Stop the previous simulation before starting another in the same domain.
In a terminal:

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh build
bash simulation.sh navigate profile:=adaptive \
  world:=/home/bedathenic/bedenic_ws/src/ugv_sim/ranger/ranger_mini/worlds/corridor_lab.world \
  map:=/home/bedathenic/bedenic_ws/src/ranger_navigation_startup/maps/corridor_lab/map.yaml \
  gui:=true rviz:=true
```

In another terminal, after AMCL becomes active:

```bash
cd /home/bedathenic/bedenic_ws/src/ranger_navigation_startup
bash simulation.sh initial_pose 0 0 0
```

Use `Nav2 Goal` in RViz. The initial goal sequence is `(3, 0, 0°)`,
`(4, 2.3, 0°)`, `(7, 3.65, 0°)`. Maintaining a 0° heading in the narrow
cross-corridor requires lateral driving. Longer straight segments permit
Ackermann motion. The `Navigation modes` display includes
`HOLONOMIC_ROTATE` for rotation in place and `HOLONOMIC_RECOVERY` for a
local return to the path. It marks Ackermann segments green and holonomic
segments orange, and displays the current mode above the robot.

To cancel a goal:

```bash
bash simulation.sh cancel
```

`corridor_lab` contains three long corridors, 1.20 m cross-passages, a
1.30 m main corridor, rooms, and ten inserted obstacles or constrictions.
The map is generated from the same geometry as the Gazebo world rather than
from a SLAM recording. `generate_corridor_world.py` reproduces the world,
map, and goal list. To test SLAM, launch the same `world` with
`simulation.sh slam`.

## Driving-mode selection

1. The global planner generates two candidates when a new plan is actually
   needed. Both use the costmap and robot footprint. The Ackermann candidate
   is evaluated with its headings and gear-change sequence; the Lattice
   candidate is evaluated for permitted mixed/holonomic execution. The shorter
   feasible candidate wins. With zero padding, estimated duration breaks numerical distance ties. In free space, the holonomic candidate joins start and goal
   with a straight line after checking the footprint.
2. Position and heading changes determine curvature and required radius
   `R = 1 / |kappa|`. The angle between the path tangent and chassis
   longitudinal axis is also checked. A lateral segment with constant heading
   has infinite radius but is still not Ackermann-feasible.
3. The local radius limit is 0.80 m; global Hybrid-A* uses 1.10 m to leave room
   for tracking corrections. On a constant-heading straight segment, the
   permitted difference between translation direction and the chassis
   longitudinal axis, forward or backward, is 0.025 rad. A larger difference
   requires lateral motion. Ackermann may reverse at a marked path point.
   The holonomic controller handles rotations in place.
4. Duration estimates include length, forward/reverse/lateral speeds, angular
   speed, chassis alignment, wheel-angle changes, and Ackermann reversal cost.
   Mixed execution is compared with fully holonomic execution of the same
   remaining path. A reference requiring Ackermann headings and gear changes
   must retain that sequence; lateral tracking of only its XY line is not
   offered. For practically equal duration estimates (0.25 s), a feasible
   Ackermann segment wins. This is a kinematic estimate, not a guarantee of
   globally shortest actual execution time.
5. The controller reevaluates the remaining segment without requesting a new
   global plan. Holonomic mode has a minimum dwell of 2 s. Ackermann islands
   shorter than 0.65 m are merged into surrounding holonomic segments. An
   Ackermann segment already being executed is not interrupted merely because
   its remaining length drops below that threshold.

Speeds used by the estimate are 0.20 m/s forward, 0.15 m/s Ackermann reverse,
0.20 m/s holonomic, 0.25 rad/s for alignment, and 1 rad/s for wheel steering.
Actual speed depends on clearance, the controller, and required wheel
alignment.

The Lattice motion set matches the local 0.80 m radius limit. Original arcs
with radii of 0.50–0.69 m were removed. The current set has 160 primitives:
128 translations (eight directions per heading) and 32 rotations in place.
The holonomic candidate can therefore negotiate a sharp turn through
translation and a separate rotation; the Ackermann candidate retains smooth
Hybrid-A* arcs with radii of at least 1.10 m.

## Stalls and replanning

Since 2026-11-03, each MPPI cycle takes a reference point up to 45 cm ahead of
the robot's projection onto the current segment. It simulates a sequence of
speeds and arcs subject to speed, acceleration, and braking limits. Reference
speed decreases near the segment end. If the full motion has a clear
footprint, the sampling seed combines 75% of that reference with 25% of the
previous solution. MPPI scores one batch of 1,200 samples per cycle. If the
reference crosses an obstacle, sampling starts from the previous solution.

At a reversal, the controller explicitly checks the case where only the
incoming segment's endpoint remains. If the robot has passed that point or
cannot achieve its requested heading within tolerance over the remaining
distance, the adaptive profile brakes. Within 15 cm, it can return
holonomically to the point and align through a checked rotation before
continuing the same path. A pure Ackermann profile instead requests a new
approach. Tracking assistance permits motion only when the reference point
lies ahead in the selected driving direction.

In a drive recorded on 2026-11-03, the initial selection was a 7.55 m mixed
path with an estimated duration of 49.46 s. The Ackermann alternative was
8.95 m with an estimate of 48.07 s. The shorter route won. That historical run used an additional 5 cm for lattice search beyond its
4 cm planning reserve.

In Gazebo, the original controller stopped following the recorded 100-pose
path at its first reversal. The updated version completed that same path in
85.1 s without collision. Tracking error was within 3.9 cm for 95% of samples.
Two local alignments were recorded before resuming Ackermann mode. Three
standalone Ackermann goals each finished with one plan. A repeated adaptive
drive to `(7.05, -1.11, 1.596 rad)` used a holonomic path, with one global
plan and a final position error of 3.0 cm.
[Measurements, recorded path, and validation results](validation/ackermann-continuity-20261103.json)

In a new terminal, inspect the latest global decision and then follow local
decisions while the adaptive profile is driving:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
ros2 topic echo /navigation_plan_decision std_msgs/msg/String --once --qos-durability transient_local
ros2 topic echo /navigation_mode_decision std_msgs/msg/String
```

Near a segment end before a reversal, MPPI can settle on almost zero speed.
DWB may oscillate between opposing commands near a new obstacle. Retained-path
tracking assistance was added: after a lack of progress, it checks a slow
geometric command, the full footprint, and the robot center along the
predicted motion. Unknown or occupied space is rejected. DWB measures progress
along the path, so forward/backward rocking cannot count as progress.
Assistance synchronizes DWB critic state with the command the robot actually
receives.

The behavior tree retains the global plan while it remains valid. A new
obstacle intersecting the plan justifies replanning; a driving-mode change
alone does not. The local controller still computes commands at 10 Hz and
must react to environmental changes. If no safe command exists, it may stop
and invoke recovery. Switching between lateral motion, rotation in place,
and curved Ackermann motion can require braking and wheel alignment; that
short stop is physically necessary.

## Rotation in place and clearance checks

The holonomic controller also follows the reference path's heading. Before
a turn, it checks the desired heading up to 2 m ahead so the chassis aligns
before entering a passage that has no room to rotate. Lookahead ends at the
next planned in-place rotation: its outgoing heading is not applied while
the robot is still approaching that point. Rotation proceeds only if a full
sweep checked in 0.025 rad increments finds no obstacle. This check repeats
during the maneuver; a newly observed obstacle stops the rotation. The robot
first brakes translation, rotates without a lateral command, and then resumes
the same global plan. Correction ends within 0.06 rad.

Planned rotations with no center displacement are mandatory waypoints. DWB
receives only the segment up to the next such point, approaches within 2.5 cm,
and completes the rotation before continuing. Projection must not skip a
rotation on overlapping outgoing/return XY sections. This applies to both
standalone holonomic and adaptive profiles.

An ongoing rotation uses a 4 cm hold threshold, matching the waypoint passage
tolerance. Small estimated-pose changes therefore do not repeatedly switch
the wheels between translation and rotation. Larger displacement interrupts
the rotation and realigns position. A mandatory rotation uses the segment's
final pose even when it lies more than 3 m from the path start. It does not
compare clearance with a corridor already passed or wait for the pause used
for optional heading changes. It still checks all space swept by the footprint.

If the current heading blocks the footprint within the next 0.6 m, alternative
headings are checked before a stall. Up to 0.8 m of the reference path is
examined. The same check runs when DWB makes no progress or finds no legal
velocity. Candidate chassis headings are the path tangent, opposite tangent,
and ±90° rotations. A change must open at least another 0.20 m of traversable
path; the controller does not immediately return to the reference heading
that would close the passage again. `PoseProgressChecker` recognizes angular
progress as well as center displacement.

At a sharp corner, reference projection cannot skip the vertex: the holonomic
controller approaches within 5 cm before changing direction. Switching from
holonomic to Ackermann mode requires tracking error no greater than 6 cm.
This avoids the previously measured 13 cm corner cut and an unsuitable
Ackermann exit from the corner. On an MPPI error caused by blocked samples,
a slow Ackermann correction is checked first; the adaptive profile can then
attempt a short holonomic return to the reference. This attempt does not skip
a forward/reverse transition.

Wheel feedback remains valid in pivot/crab positions. The absence of a finite
Ackermann radius in those positions must not produce a false
`Missing or stale steering joint feedback` error. Actual missing or stale
`joint_states` still stops the Ackermann controller.

The global planner additionally checks the full goal footprint and interpolated
poses of both candidates after native planning/smoothing. It does not send
the controller a path that already fails collision checks. The local and global costmaps, goal and retained-path checks, lattice search,
shortcuts and motion guard use the same unpadded polygon. Its Gazebo dimensions
are 0.64 × 0.60 m. The Jazzy hardware configuration uses the stock Mini 2.0
dimensions of 0.738 × 0.500 m, centered on `ranger_base_link`.
Inflation costs still guide trajectories near obstacles.

The threshold for detailed Lattice footprint checks is calculated from the
search footprint's circumscribed circle, including a cell diagonal. In
more open space, the center-cell cost gradient remains active and favors
the corridor center. Always using the maximum boundary cost flattened
traversable positions in a narrow corridor and, in a repeated test, selected
a path too close to the wall.

Earlier tests with a planning reserve rejected the goal
`(13.7986, 5.83077, 0)` beside an obstacle in `corridor_lab`. Use `(13.8, 6.0, 0)` for the final-passage test; the planner
does not automatically relocate the user's goal.

The adaptive profile uses a final XY tolerance of 0.06 m, aligned between
the goal checker and DWB. The former 0.15 m tolerance allowed stopping before
a goal near an obstacle, leaving the next plan's start without tracking
clearance. This tolerance applies to the localized map pose; deviation from
the actual Gazebo pose is measured separately in the test.

The adaptive goal check has `stateful: false`: XY position and final heading
must be within tolerance at the same time. On the last at most 0.30 m of a
one-way path, within 0.18 m of the goal, the final approach switches to
holonomic mode. It corrects position first, then rotates with a full-footprint
check. An Ackermann alignment arc therefore cannot carry the robot away
from an already reached position. This transition does not skip a planned
pivot, reversal, or remaining loop section that merely passes near the goal.

The adaptive Lattice planner also validates the final connection to the
exact requested pose. If snapping position or heading to the lattice blocks
the goal, it searches for a traversable intermediate pose within
`GridBased.goal_connection_distance` (0.25 m). There it can rotate the
chassis, then translate at constant heading to the original goal. The entire
rotation and connection are checked with the planning reserve; an occupied
goal is still rejected. This is not a tolerance that moves the goal:
the last plan pose keeps the requested coordinates and orientation.

Marking new LiDAR obstacles is limited to 3 m; raytracing clears up to 3.5 m.
A long test revealed shifted cells on distant walls that blocked an otherwise
traversable goal. The static map still covers the entire world; these limits
apply only to costmap observations, not SLAM or AMCL sensor range. The
behavior-tree timeout increased from 200 ms to 3,000 ms so two planning
searches would not cause a premature service-call or cancellation timeout.

To record a long test after initializing a fresh corridor simulation in
domain 88:

```bash
RANGER_SIM_DOMAIN=88 bash simulation.sh validate_long_route \
  --goal '[13.8,6.0,0.0]' --output /tmp/ranger-long-route.json
```

The test saves poses, commands, mode changes, actual Gazebo rotations,
replanning reasons, and both costmap snapshots. The intended result includes
more than `Goal succeeded`: collision absence and absence of simultaneous
lateral and angular velocity must also be checked, since this base mode does
not execute them together.

## Diagnostics

- `/navigation_mode`: current mode, including alignment and braking.
- `/navigation_mode_decision`: radius (`null` means straight/infinite),
  geometric feasibility, time estimates, and mode-switch count.
  `holonomic_eta: null` means the reference requires an Ackermann pose sequence.
- `/navigation_plan_decision`: selected global candidate and its estimates.
- `/navigation_modes`: RViz MarkerArray.
- `/plan`, `/local_plan`, `/cmd_vel`, `/odometry/filtered`: path and tracking.

Diagnostic ROS commands must use the same installed overlay and
`ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1` as the interactive simulation.
Automatic validation runs exclusively in separate domain 88.

## Reproducible validation

After starting a **fresh** corridor simulation with `RANGER_SIM_DOMAIN=88`
and sending the initial pose in the same domain:

```bash
RANGER_SIM_DOMAIN=88 bash simulation.sh validate_tracking \
  --profile adaptive --require-mode-switch \
  --world /home/bedathenic/bedenic_ws/src/ugv_sim/ranger/ranger_mini/worlds/corridor_lab.world \
  --goals '[[3.0,0.0,0.0],[4.0,2.3,0.0],[7.0,3.65,0.0]]' \
  --output /tmp/ranger-corridor-results.json
```

The test measures actual motion in both modes, switching within the same
goal, path/localized-pose differences, footprint collisions against Gazebo
geometry, command limits, TF parents, and stopping after cancellation.
`validate_path_retention --profile adaptive` separately checks a new obstacle
in `slam_lab`: it expects an initial plan and one justified replan, without
`FollowPath` recovery. That test must also begin from a fresh simulation.

`bash simulation.sh test` runs automatic world, geometry, and base-control
checks. Enable C++ regressions by building `ranger_nav_plugins` with
`-DBUILD_TESTING=ON`, then run them through CTest.


## Logged goal and rotation sequencing (2026-11-01)

The initial `goal is occupied with no tolerance` rejection for
`(14.3401, 5.96356, -1.03037)` was fixed: lattice snapping mapped a
traversable requested pose to a blocked discrete pose. The final connection
now retains the exact goal and validates the complete rotation/approach.
Checks were added for rotating before a stall, honoring planned rotation
locations, using the actual final waypoint on long paths, and excluding
mandatory rotations from the pause for optional heading changes.

**The full drive to that goal has not yet been confirmed successful.**
The initial reproduction was interrupted before motion. After the fixes,
the robot followed one plan and reached the final rotation, but new LiDAR
observations marked the final path boundary occupied. `IsPathValid`
interrupted the action; there was no `FollowPath` failure or detected
collision. In the last trial with the rotation hold threshold, tracking RMS
was 1.35 cm and the final pose was 2.81 cm from the goal with 0.493 rad
(28.3°) of remaining heading error. The action ended with status 6 after
237.1 s; the goal was not reached.

In the preceding trial, before the rotation hold threshold, the first path
invalidation occurred when two cells near the upper wall, centered at
`(14.475, 6.425)` and `(14.525, 6.425)`, changed cost from 253 to 254.
The geometric wall boundary is `y=6.5`. Current localization and LiDAR
observations therefore do not provide a stable reserve for that tight final
rotation. The 0.02 m execution padding and additional 0.04 m planning reserve
were not reduced. Scan/map alignment near the wall needs a separate check
before this pose can be used reliably. A later attempt to leave that tight
final pose for a control goal was also rejected during planning and retained
as a separate failed trial.

Raw before/after results, intermediate trials, and costmap snapshots are
preserved under `validation/navigation-goal-*-20261101*`.
`navigation-goal-clearance-20261101.json` records the cells that invalidated
the final approach. Rotation and new-obstacle control tests are reported
separately; their success does not replace the failed original-goal trial.

A separate control goal `(0, 0, pi/2)` from a fresh simulation completed in
36.4 s with one plan, no collision, and no behavior-tree failure. Final map
position error was 1.86 cm and heading error was 0.141 rad (8.1°, within the
configured 0.15 rad tolerance). Before the rotation hold threshold, the same
test did not finish within 90 s and had 55 mode changes; after the fix, 13
changes were recorded. This is an improvement, not a claim that every stop
was eliminated. Records are
`navigation-goal-rotation-before-hysteresis-20261101.json` and
`navigation-goal-rotation-control-20261101.json`.

These trials predate the final goal-check refinement. That refinement was
added after a goal-preemption test exposed a false `SUCCEEDED`: with a
`stateful` check, the robot finished 0.56 m from the new goal after
Ackermann alignment. The failed record is
`navigation-goal-before-final-alignment-20261101.json`.

The final regression sequence on the latest libraries passed in a fresh
`slam_lab` world: an unchanged goal and through-poses each used one plan;
an inserted obstacle required two plans, a 0.947 s reaction, and no
`FollowPath` recovery; goal preemption used two plans and finished with
5.48 cm XY error and 0.0044 rad heading error. Cancellation returned
`CANCELED`, a zero command, and a stationary robot. No collision was
recorded. The record is `navigation-goal-obstacle-20261101.json`.

All 50 C++ and 24 Python checks passed. Tests ran without a GUI in isolated
domain 88; no physical robot was tested. The source/installed-library manifest
is `validation/navigation-goal-summary-20261101.json`. Libraries were
installed into the default `.runtime`; completely restart the navigation
launch to load the changes.

![Last attempt at the original goal, stopped before completion](validation/navigation-goal-20261101.png)

## Rotation-update validation (2026-10-31)

Gazebo Classic ran without a GUI in isolated ROS domain 88. Two successful
goals were executed in the same simulation without resetting localization
between drives:

| Goal `(x, y, yaw rad)` | Duration | Global plans | Tracking RMS | Final map error |
| --- | --- | --- | --- | --- |
| `(13.7986, 6.0, 0)` | 206.0 s | 1 | 1.60 cm | 8.95 cm |
| `(13.85, 4.45, -1.54592)` | 70.2 s | 1 | 1.59 cm | 9.05 cm |

Both drives completed without `FollowPath`/`IsPathValid` failures,
detected footprint collisions, or simultaneous lateral/angular commands.
The second included measured in-place rotation totaling 1.577 rad,
approximately 90°. The first route needed no chassis rotation: traversable
lateral driving remains permitted. Final errors against the Gazebo pose were
5.3 and 7.1 cm. These are individual drives, not a statistical guarantee of
repeatability or optimal duration.

The original second goal `(14.5994, 4.5625, -1.54592)` after the long route
was rejected because the observed footprint lacked obstacle clearance.
The trial pose `(14.5, 4.35, -1.54592)`, which intersects the lower wall,
was also rejected. These rejections do not count as successful navigation;
they caused no motion between the two successful drives. An earlier isolated
test of the original second goal completed in 44.1 s with approximately 94°
of rotation, but that does not confirm clearance after new LiDAR observations.
Use the verified pose `(13.85, 4.45, -1.54592)` above to repeat the final
maneuver.

All 40 C++ and 24 Python checks passed. A planner crash on exit was also
fixed: Humble Hybrid and Lattice load in separate plugin scopes, avoiding a
double free of shared static data. The final `planner_server` shut down
cleanly after navigation.

A repeated `slam_lab` test confirmed one global plan for an unchanged goal
and one for through-poses. An obstacle inserted during driving caused one
justified replan after 0.962 s; the goal was reached with two plans total,
without `FollowPath` recovery or collision. Goal preemption and cancellation
with the robot stopping also passed. The record is
`validation/navigation-rotation-obstacle-20261031.json`.

Data: `validation/navigation-long-rotation-final-20261031.json`,
`validation/navigation-second-after-route-20261031.json`, and
`validation/navigation-second-user-goal-after-route-20261031.json`.
Earlier failed trials and results before the cost-gradient/final-tolerance
fix remain stored separately. Parameters and clearance validation apply to
this simulation; physical-robot behavior was not tested. The validation
summary and source/installed-library SHA-256 hashes are in
`validation/navigation-rotation-summary-20261031.json`.
Verified libraries were installed in the default `.runtime`; completely
restart the navigation launch to load changes.

![Long route and measured rotation](validation/navigation-rotation-20261031.png)

## Earlier validation, before the rotation update (2026-10-31)

All trials below ran without a GUI in domain 88. The RViz configuration
includes a new driving-mode display; this validation was not a visual GUI
test.

- Standalone Ackermann: three reference goals, one plan per goal. The
  previously problematic third maneuver decreased from 104.4 s/two plans
  to 62.5 s/one plan, with 2.1 cm RMS error. No collisions occurred.
- Standalone holonomic new-obstacle test: an initial plan and one replan after
  the path was blocked; 0.95 s reaction and no `FollowPath` recovery.
  Through-poses, goal preemption, and cancellation also passed.
- The adaptive new-obstacle test also passed with two plans total, a 0.96 s
  reaction, and no `FollowPath` recovery. Through-poses, goal preemption,
  and cancellation passed without collisions.
- Adaptive `corridor_lab`: G1/G2/G3 completed in 29.5/31.2/45.8 s, each with
  one plan. Tracking RMS was 1.8/3.8/5.0 cm; maximum deviation in the third
  drive was 15.6 cm. Both modes were confirmed through measured motion,
  including a switch within the third goal. There were zero footprint
  collisions and zero Ackermann-radius violations; cancellation stopped the
  robot.

An additional adaptive goal `(1.2, 1.2, 90°)` was executed in Ackermann mode
in 16.2 s, with one plan and 2.0 cm RMS error. This also checked a curved
Ackermann heading sequence in the adaptive controller.

RMS describes path tracking rather than final goal accuracy. In this profile,
final map-pose error is approximately 16–18 cm. ETA is a nominal model:
for example, the initial estimate for the straight goal was 15.6 s, while
actual duration was 29.5 s. The model does not prove that the selected
execution is time-optimal. Wheel alignment when switching between crab
directions and driving modes still requires short stops. Not every narrow
passage combination or continuously moving obstacle has been validated.

Data: `validation/navigation-progress-ackermann-20261031.json`,
`validation/navigation-progress-obstacle-20261031.json`,
`validation/navigation-adaptive-corridor-20261031.json`, and
`validation/navigation-adaptive-obstacle-20261031.json`.
Earlier attempts remain stored as `trial1` and `trial2`; `trial2`
records the rejected estimate of holonomic tracking for an Ackermann
reference with a reversal.

![Measured drive](validation/navigation-adaptive-corridor-20261031.png)

![Full world layout](validation/corridor-lab-layout-20261031.png)
