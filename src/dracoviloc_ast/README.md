# dracoviloc_ast

AST consumes ODAS `/sss` audio and `/sst` tracks. When it classifies a track
as a drone, it publishes that track's direction on `/ast/direction` as a
`geometry_msgs/Vector3Stamped`. Models are loaded from `models/ast/`.
