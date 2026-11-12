# Ranger autonomy Jazzy R7

Image: `ghcr.io/bedathenic/ranger-autonomy:jazzy-r7` for Linux/amd64.
GitHub Actions builds, tests and publishes the same image in one job.

Adaptive navigation can opt into a bounded footprint escape for a blocked
turn or approach. The hardware test procedure enables it and records
`/footprint_escape_status` alongside the scans, costmaps and command stages.

The Jazzy CI tests cover adaptive recovery enabled and disabled, holonomic and
Ackermann navigation, the drive gateway and virtual CAN steering. The
blocked-turn fixture uses synthetic sensors and a stationary base. Physical
robot validation is pending.

Download **ROBOT-TEST.md** for the R6 upgrade, RViz test, bag export and rollback.
