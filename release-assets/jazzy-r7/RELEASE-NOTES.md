# Ranger autonomy Jazzy R7

Image: `ghcr.io/bedathenic/ranger-autonomy:jazzy-r7` for Linux/amd64.
Image ID: `sha256:1a91eca9812d2f42f5878c9a44960451537e2eed01ced1aaa71a6b2b4e46e3c4`.
The image was built from the navigation and deployment snapshot at
`f9b696fc77cd811828494cc1358d7eeb6b4542ab`.

Adaptive navigation can opt into a bounded footprint escape for a blocked
turn or approach. The hardware test procedure enables it and records
`/footprint_escape_status` alongside the scans, costmaps and command stages.

The local Jazzy tests covered adaptive recovery enabled and disabled,
holonomic and Ackermann navigation, the drive gateway, virtual CAN steering,
and MCAP recording. The blocked-turn fixture used synthetic sensors and a
stationary base. Physical robot validation is pending.

Download **ROBOT-TEST.md** for the R6 upgrade, RViz test, bag export and rollback.
