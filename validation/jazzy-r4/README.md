# jazzy-r4 build validation — 2026-11-03

The current local planner/controller sources, tests, navigation profiles and
omnidirectional lattice were synchronized into this deployment repository.
Robot message definitions and unrelated simulation/driver changes were retained.

- Native Humble planner/controller rebuild: passed; colcon reports 71 tests, no failures or skips.
- amd64 Jazzy image rebuild: passed.
- Original Jazzy navigation regressions: colcon reports 71 tests, no failures, one intentional single-goal waypoint-pruning skip.
- Original Python logging regressions: 11 passed.
- Isolated Compose integration: all Nav2 lifecycle nodes active, planning action successful, scan conversion and structured logging verified; all eight services exited normally.
- Drive gateway integration: passed, including permitted competing publishers and ignored arm expiry.
- Passive Ackermann diagnostic: passed with synthetic ROS and real SocketCAN on vcan in a network namespace without robot interfaces.
- 42 package/config hashes in the image manifest match the synchronized files.

Machine-readable results are in the adjacent JSON files. These are software
tests with synthetic input. Physical adaptive/Ackermann navigation and steering
calibration remain unverified. The existing jazzy-r3-gui viewer uses compatible
ROS interfaces and does not require a rebuild for these planner/controller changes.
