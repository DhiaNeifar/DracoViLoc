# dracoviloc_ekf

C++ extended Kalman filter for DracoViLoc direction estimates. It optionally
subscribes to `/yolo/direction`, `/ast/direction`, and `/gre/direction`; every
input and the `/ekf/direction` output uses
`geometry_msgs/Vector3Stamped` with `(x,y,z)` as a direction vector.

`/ekf_fused_target_pose` is temporarily retained as a compatibility alias and
will be removed after downstream users migrate to `/ekf/direction`.

`output_average_window` controls a causal normalized-vector average applied to
accepted EKF estimates before publication. The default is 5 samples; set it to
1 to publish the unaveraged EKF estimate.

`yolo_measurement_noise` and `mobilenetv2_measurement_noise` allow the visual
and acoustic directions to use different covariance. A value at or below zero
falls back to the common `measurement_noise`. The main bringup defaults to
0.03 for YOLO and 0.15 for MobileNetV2/ODAS because the latter is a noisier,
intermittent bearing. Sensor timestamps are kept monotonic when callbacks from
the two pipelines arrive out of order.

The EKF never subscribes to `/sst`. AST and GRE own the association between
their classification result and the corresponding ODAS direction.
