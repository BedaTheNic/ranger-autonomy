# jazzy-r5 software validation — 2026-11-03

The release synchronizes the latest local planner, tracking, wheel-control,
logging and navigation configuration sources. It adds the separate hardware
motion-guard service and retained-path service remapping used by those sources.

The drive gateway forwards accepted planar velocities unchanged. The former
0.05 m/s and 0.10 rad/s clamp, hard commissioning ceilings and DRIVE_MAX_*
settings are removed. Configured controller limits and the developed
stopping-distance guard remain, as do arming, command timeout, odometry,
system-state and measured-steering checks.

Validation:

- Native planner rebuild and 76 reported navigation tests: passed, no skips.
- Native wheel-control/kinematics/odometry Python regressions: 19 passed.
- amd64 Jazzy image build: passed; 76 reported navigation tests passed with
  one existing, intentional single-goal waypoint-pruning skip; 11 logging tests passed.
- Adaptive, holonomic and standalone Ackermann Compose integrations: passed.
  They verify scan conversion, lifecycle activation, planning, retained-path
  validation, real controller/guard topic routing, stale-scan stopping, action
  cancellation, structured logging and normal container shutdown.
- Gateway regression: forward/reverse/diagonal commands above the former speed
  cap pass unchanged, even with legacy speed environment variables present.
  Command timeout, E-stop, stale odometry, goal freshness, source qualification
  and required adaptive steering feedback remain tested.
- 79 package/config hashes match the image manifest and local source snapshot.

These tests use synthetic inputs in isolated networks and do not establish
physical adaptive/Ackermann navigation or wheel calibration. The existing
jazzy-r3-gui viewer remains compatible. The initial multi-package native build
stalled before its compiler started; the planner was rebuilt successfully
using its existing workspace overlay.
