# dracoviloc_ast

AST consumes ODAS `/sss` audio and `/sst` tracks. When it classifies a track
as a drone, it publishes that track's direction on `/ast/direction` as a
`geometry_msgs/Vector3Stamped`. Models are loaded from `models/ast/`.

A bearing gate (enabled by default, `--bearing-gate true|false`) protects the
published direction against ODAS track retargeting: ODAS can keep a track id
while its bearing jumps to a wall reflection. A classified bearing more than
`--max-bearing-jump-deg` (60 deg) from the last published one is held back
unless a consistent challenger confirms (`--bearing-confirm`, 3 consecutive
windows in a 15 deg cone); after `--bearing-timeout` (2 s) the gate re-arms.
GRE and MobileNetV2 apply the same gate on their direction topics.
