# dracoviloc_ekf

C++ extended Kalman filter for DracoViLoc direction estimates. It optionally
subscribes to `/yolo/direction`, `/ast/direction`, and `/gre/direction`; every
input and the `/ekf/direction` output uses
`geometry_msgs/Vector3Stamped` with `(x,y,z)` as a direction vector.

`/ekf_fused_target_pose` is temporarily retained as a compatibility alias and
will be removed after downstream users migrate to `/ekf/direction`.

The EKF never subscribes to `/sst`. AST and GRE own the association between
their classification result and the corresponding ODAS direction.
